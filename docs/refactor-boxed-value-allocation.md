# Refactor note: eliminating the pooled allocation behind boxed value types

*Written 2026-08-11, after embedded value types landed. Nothing here is urgent — this is
the next step if the script binding's allocation traffic ever needs to come down further.*

## Where things stand

A value returned from a bound method to script land currently costs, per value:

1. **one pooled C++ allocation** — `CastAny< Vector4 >::set(v)` does
   `Any::fromObject(new BoxedVector4(v))`, and `Boxed` allocates from `BoxedAllocator`;
2. **one script-side allocation** — the Lua userdata;
3. **one copy** — `IRuntimeClass::embedCopy` copy-constructs the box into that userdata;
4. **one pooled free** — when the `Any` goes out of scope at the end of the dispatch.

Steps 2–4 are the cheap part. Step 1 is what is left to remove, and it is the only one
that touches a heap.

What already went away (see `ScriptManagerLua::pushValueInstance`,
`AutoRuntimeClass::setEmbeddedValueType`): the box no longer *lives* on the pool while script
land holds it, and the wrapper carries no `__gc`. That was the important fix — Lua runs
finalizers at a bounded rate per cycle and could not keep up with millions of finalizable
objects a minute, which is what made a long session's heap climb to 129 MiB and made
leaving a stage stall for seconds.

Note the *argument* direction (script → C++) never allocated: `toAny` on a userdata does
`Any::fromObject(<pointer to the embedded box>)`, which only takes a reference. **This
refactor is only about values going C++ → script.**

## What it is worth

Measured with a standalone harness against vendored Lua 5.5, 3M pushes (roughly a minute
of play in the Zombie game at ~45k boxed values a second):

| representation | time | live heap | pooled allocs |
| --- | --- | --- | --- |
| pointer + `__gc`, incremental collector *(the original bug)* | 583 ms | 58 MiB | 3M |
| pointer + `__gc`, generational collector | 239 ms | 0.02 MiB | 3M |
| **embedded, no finalizer** *(today)* | **121 ms** | 0.02 MiB | 3M |
| embedded, no finalizer, constructed in place *(this refactor)* | **97 ms** | 0.02 MiB | **0** |

So roughly a further 20% off the push path, and three million pooled allocate/free pairs a
minute stop happening. In frame terms that is small — order 10 µs a frame at the rates
above — so do this for the allocator pressure and the simplicity, not for frame time.

## Why the obvious approach does not work

The tempting move is to make `CastAny< T >::set` hand back a *borrowed* pointer — a
`{ TypeInfo, const void* }` pair aimed at the caller's own value — and let the script
backend construct straight from it, boxing nothing.

It dangles. Look at the shape of a dispatcher (`Core/Class/AutoMethod.h`):

```cpp
ReturnType result = (self->*m_method)(...);   // a local
return CastAny< ReturnType >::set(result);    // ...whose address would escape
```

`result` dies when `invoke` returns, *before* the caller ever pushes the `Any`. Boxing is
exactly what buys the value the extra lifetime it needs. So any fix has to give the value
somewhere to live that outlives the dispatcher's frame — which means **storage owned by
the `Any` itself**.

## The design that does work: small-object storage in `Any`

Give `Any` an inline buffer and construct the box into it instead of onto the pool.

### Representation

`Any` today is a `Type` tag plus a union of `bool/int32/int64/float/double/char*/ITypedObject*`
— about 16 bytes. Add one kind:

```cpp
enum class Type : uint8_t { ..., Object, ObjectInline };

struct ValueOps                       // one static instance per embeddable box type
{
    void (*copy)(void* dst, const void* src);
    void (*destroy)(void* at);        // may be null when there is nothing to do
};

// in the union / alongside it
struct
{
    const ValueOps* ops;
    alignas(16) uint8_t storage[c_anyInlineSize];
} m_inline;
```

`getObject()` / `getObjectUnsafe()` return `(ITypedObject*)m_inline.storage` for
`ObjectInline`. **That is the whole reason this is tractable**: every consumer — including
`ScriptManagerLua::pushValueInstance`, `toAny`, and every `CastAny< T >::get` — keeps
working unchanged, because they only ever read an `ITypedObject*` and unbox it.

An ops table rather than `memcpy` because a box has a vtable, so it is not
*trivially copyable* by the letter of the standard even though the payload is plain data.
Two function pointers per type is cheaper than arguing about it.

### `CastAny< T >::set`

```cpp
static Any set(const Vector4& value) {
    return Any::fromInlineValue< BoxedVector4 >(value);   // constructs in place
}
```

`fromInlineValue< BoxType >` placement-constructs when `sizeof(BoxType) <= c_anyInlineSize`
and falls back to `fromObject(new BoxType(value))` otherwise, so nothing has to be
special-cased per call site. Use `::new (at) BoxType(...)` — the global placement form —
because `Boxed`/`Object` declare their own `operator new` and that hides the placement
overload.

### Sizing the buffer

Measured box sizes and alignments (x86-64):

| box | size | align | | box | size | align |
| --- | --- | --- | --- | --- | --- | --- |
| `BoxedColor4ub` | 16 | 8 | | `BoxedRay3` | 48 | 16 |
| `BoxedVector2` | 24 | 8 | | `BoxedBezier3rd` | 48 | 8 |
| `BoxedVector4` | 32 | 16 | | `BoxedMatrix33` | 64 | 16 |
| `BoxedQuaternion` | 32 | 16 | | `BoxedMatrix44` | 80 | 16 |
| `BoxedColor4f` | 32 | 16 | | `BoxedIntervalTransform` | 96 | 16 |
| `BoxedAabb2` | 32 | 16 | | `BoxedFrustum` | **560** | 16 |
| `BoxedGuid` | 32 | 8 | | | | |
| `BoxedTransform` | 48 | 16 | | | | |
| `BoxedAabb3` | 48 | 16 | | | | |
| `BoxedPlane` | 48 | 16 | | | | |
| `BoxedSphere` | 48 | 16 | | | | |

**48 bytes is the recommended budget.** It covers everything the hot path actually pushes
(`Vector4`, `Transform`, `Quaternion`, `Color4f`, the AABBs, `Plane`, `Sphere`, `Ray3`,
`Guid`) and puts `Any` at ~64–72 bytes. The matrices, `IntervalTransform` and `Frustum`
keep the heap path; they are rare, and `Frustum` at 560 bytes would be absurd inline.

Check the cost before committing: the dispatch path holds `Any argv[8]` and `Any argv[10]`
on the stack (`ScriptManagerLua`), so a 72-byte `Any` means ~720 bytes a call rather than
~160. Fine, but measure a deep script call chain rather than assuming.

### Contract change, and the invariant it creates

For an `ObjectInline` any, **`getObject()` returns a borrowed pointer that dies with the
`Any`**. Today a value-type `Any` hands out a reference-counted pointer that may legally
outlive it. Nothing in the current code retains one — `CastAny< T >::get` unboxes
immediately, and the script backends copy into their own storage — but that goes from
"true by construction" to "true by convention".

Make it enforceable rather than assumed:

- grep every `getObject()` / `getObjectUnsafe()` call site for one that stores the result
  in a `Ref`, a member, or a container;
- in debug, have `addRef` on an inline instance assert. It is already the case for embedded
  script-side instances (`pushValueInstance` keeps one reference that is never released so
  the counter can never reach zero and hand Lua-owned storage to the allocator); the same
  trick applies here.

### Move semantics

`Any`'s move constructor currently steals the union and marks the source `Void`. Inline
storage cannot be stolen — the move has to copy the bytes through `ops->copy` and destroy
the source. Cheap for these types, but it *must* be written; a defaulted move would leave
two objects pointing at the same storage and destroy it twice.

## Suggested order of work

1. Add `ValueOps`, the `ObjectInline` kind, and `Any::fromInlineValue< T >`, with
   copy/move/destroy handled. Keep everything else on the existing path — no `CastAny`
   changes yet. Ship it: no behaviour should change at all.
2. Switch **one** `CastAny` specialisation (`Vector4`) to `fromInlineValue`. Run the game.
   `Vector4` is the highest-traffic type, so if the contract is wrong anywhere it will
   surface here first and fastest.
3. Switch the rest of the 48-byte-and-under boxes. Leave the matrices, `IntervalTransform`
   and `Frustum` alone.
4. Confirm the pool is quiet: `BoxedAllocator`'s traffic for these types should drop to
   zero. A counter in `BoxedAllocator::alloc` under `_DEBUG` is the easiest proof.
5. Re-measure against the harness numbers above (97 ms, zero pooled allocations).

## Related code

| what | where |
| --- | --- |
| `Any` representation | `code/Core/Class/Any.h`, `Any.cpp` |
| value conversion | `code/Core/Class/CastAny.h` and each `code/Core/Class/Boxes/Boxed*.h` |
| return-value boxing | `code/Core/Class/AutoMethod.h`, `AutoProperty.h`, `AutoOperator.h`, `AutoStaticMethod.h` |
| embeddable-class plumbing | `code/Core/Class/IRuntimeClass.h`, `RuntimeClass.*` (holds the size and copy thunk), `AutoRuntimeClass.h` (`setEmbeddedValueType`, non-virtual — see below) |
| which boxes are embeddable | `code/Core/Class/BoxedClassFactory.cpp` (18 of 19; `BoxedRange` keeps its destructor — it holds its bounds as `Any`) |
| script-side storage | `ScriptManagerLua::pushValueInstance`, `code/Script/Lua/ScriptManagerLua.cpp` |
| the pool being avoided | `code/Core/Class/BoxedAllocator.h` |

## Two things not to lose along the way

- **Alignment is handled in the payload, deliberately.** Lua's `LUAI_MAXALIGN` gives
  userdata 8-byte alignment while most of these boxes want 16, so `getEmbeddedSize()`
  reserves `sizeof(T) + alignof(T) - 1` and `embedCopy` aligns up inside it. That is why no
  change to vendored Lua was needed, and why the failure mode of getting it wrong is a
  crashing SIMD load rather than a wrong number. `Any`'s inline buffer needs the same care
  (`alignas(16)` is enough there, since `Any` itself is then 16-aligned).
- **The collector mode stopped mattering once the finalizers went away** (121 ms
  generational vs 140 ms incremental, against 239 / 583 before). Keep it that way: any new
  wrapper type that carries a `__gc` and is created per frame re-opens the original problem.
- **Keep type-dependent code out of virtual members of `AutoRuntimeClass`.** Virtual members
  of a class template are instantiated *with the class*, called or not, because the vtable
  needs them. A virtual `embedCopy()` therefore forced an implicit copy constructor into
  existence for every class ever wrapped in an `AutoRuntimeClass` — and broke the link of
  `Traktor.Run.App`, where `run::ProduceOutput` is nominally copy constructible
  (`std::is_copy_constructible_v` is true, since the implicit constructor is *declared*) but
  copying it needs a `StringOutputStreamBuffer` vtable that no translation unit emits. Note
  what this means for the trait: `is_copy_constructible_v` answers a question about
  declarations, not about whether the copy will compile and link. The opt-in is consequently
  a plain member function calling `RuntimeClass::setEmbeddedValue(size, thunk)`, instantiated
  only where called — the eighteen calls in `BoxedClassFactory.cpp` — with `static_assert`s
  there to reject an abstract or non-copyable class outright rather than degrade silently.
