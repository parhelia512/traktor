/*
 * TRAKTOR
 * Copyright (c) 2026 Anders Pistol.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "Terrain/GrassComponent.h"

#include "Core/Log/Log.h"
#include "Core/Math/Const.h"
#include "Core/Math/Half.h"
#include "Core/Math/Quasirandom.h"
#include "Core/Math/RandomGeometry.h"
#include "Heightfield/Heightfield.h"
#include "Render/Buffer.h"
#include "Render/Context/RenderContext.h"
#include "Render/IRenderSystem.h"
#include "Render/VertexElement.h"
#include "Resource/IResourceManager.h"
#include "Terrain/GrassComponentData.h"
#include "Terrain/Terrain.h"
#include "Terrain/TerrainComponent.h"
#include "Terrain/TerrainSurfaceCache.h"
#include "World/Entity.h"
#include "World/Entity/DisplacementWorldComponent.h"
#include "World/IWorldRenderPass.h"
#include "World/World.h"
#include "World/WorldBuildContext.h"
#include "World/WorldRenderView.h"

#include <limits>

namespace traktor::terrain
{
namespace
{

//! Number of vertical segments in a single blade; the top segment converges into the tip vertex.
const int32_t c_bladeSegments = 4;
const int32_t c_bladeVertexCount = 2 * c_bladeSegments + 1;
const int32_t c_bladeTriangleCount = 2 * ((c_bladeSegments - 1) * 2 + 1); // Both windings.

//! Number of heightfield grid cells spanned by a cluster of blades, in each direction.
const int32_t c_clusterGridSize = 16;

#pragma pack(1)

struct Vertex
{
	float position[2]; //!< Lateral offset and normalized height of blade.
	half_t texCoord[2];
};

#pragma pack()

#pragma pack(1)

struct BladeData
{
	float positionX;
	float positionZ;
	float rotation;
	float dummy1;
	float scale;
	float random;
	float dummy2;
	float dummy3;
};

#pragma pack()

const render::Handle s_handleTerrain_Normals(L"Terrain_Normals");
const render::Handle s_handleTerrain_Heightfield(L"Terrain_Heightfield");
const render::Handle s_handleTerrain_SurfaceAlbedo(L"Terrain_SurfaceAlbedo");
const render::Handle s_handleTerrain_WorldExtent(L"Terrain_WorldExtent");
const render::Handle s_handleGrass_Eye(L"Grass_Eye");
const render::Handle s_handleGrass_MaxDistance(L"Grass_MaxDistance");
const render::Handle s_handleGrass_Blades(L"Grass_Blades");
const render::Handle s_handleGrass_Order(L"Grass_Order");

Vertex packVertex(float x, float y, float u, float v)
{
	Vertex vtx;
	vtx.position[0] = x;
	vtx.position[1] = y;
	vtx.texCoord[0] = floatToHalf(u);
	vtx.texCoord[1] = floatToHalf(v);
	return vtx;
}

//! True if an attribute carry any density within a grid rectangle.
bool anyAttributeDensity(const hf::Heightfield* heightfield, int32_t gridX, int32_t gridZ, int32_t gridSize, uint8_t attribute)
{
	for (int32_t z = 0; z < gridSize; ++z)
	{
		for (int32_t x = 0; x < gridSize; ++x)
		{
			if (heightfield->getGridAttributeDensity(gridX + x, gridZ + z, attribute) != 0)
				return true;
		}
	}
	return false;
}

/*! Scatter blades of a single grass type across a cluster.
 *
 * Each candidate is accepted with a probability equal to the painted attribute density
 * at it's own position, thus both the number and the size of the blades follow the
 * density at grid cell resolution instead of being constant across the entire cluster.
 *
 * Deterministic in the cluster center and the heightfield alone; it's called once to
 * count the accepted blades and later once more to write them. Should the heightfield
 * have been repainted in between the passes disagree, thus writing is capped.
 *
 * \return Number of accepted blades, of which the first \a maxBlades are written to
 * \a outBlades when given.
 */
int32_t scatterCluster(
	const hf::Heightfield* heightfield,
	const Vector4& center,
	float clusterSize,
	uint8_t attribute,
	int32_t candidateCount,
	float scale,
	BladeData* outBlades,
	int32_t maxBlades)
{
	RandomGeometry random(int32_t(center.x() * 919.0f + center.z() * 463.0f));

	// A single shift for the entire set decorrelate it from neighbouring clusters
	// without disturbing the stratification within it.
	const Vector2 shift(random.nextFloat(), random.nextFloat());

	int32_t count = 0;
	for (int32_t i = 0; i < candidateCount; ++i)
	{
		const Vector2 ruv = Quasirandom::hammersley((uint32_t)i, (uint32_t)candidateCount, shift);

		const float px = center.x() + (ruv.x * 2.0f - 1.0f) * clusterSize;
		const float pz = center.z() + (ruv.y * 2.0f - 1.0f) * clusterSize;

		const float density = heightfield->getWorldAttributeDensity(px, pz, attribute);

		// Draw every random value unconditionally, thus the stream stay in lockstep
		// between the counting and the writing pass.
		const float accept = random.nextFloat();
		const float rotation = random.nextFloat();
		const float size = random.nextFloat();
		const float variation = random.nextFloat();

		if (accept >= density)
			continue;

		if (outBlades != nullptr && count < maxBlades)
		{
			BladeData& bd = outBlades[count];
			bd.positionX = px;
			bd.positionZ = pz;
			bd.rotation = rotation * TWO_PI;
			bd.dummy1 = 0.0f;
			// Blades shrink as the density fade out, thus a thinned out region also
			// read as shorter grass instead of only fewer blades.
			bd.scale = scale * (0.5f + 0.5f * density) * (0.5f + 0.5f * size);
			bd.random = variation;
			bd.dummy2 = 0.0f;
			bd.dummy3 = 0.0f;
		}

		++count;
	}

	return count;
}

}

T_IMPLEMENT_RTTI_CLASS(L"traktor.terrain.GrassComponent", GrassComponent, TerrainLayerComponent)

bool GrassComponent::create(
	resource::IResourceManager* resourceManager,
	render::IRenderSystem* renderSystem,
	const GrassComponentData& layerData)
{
	m_layerData = layerData;

	if (!resourceManager->bind(m_layerData.m_shader, m_shader))
		return false;

	AlignedVector< render::VertexElement > vertexElements;
	vertexElements.push_back(render::VertexElement(render::DataUsage::Position, render::DtFloat2, offsetof(Vertex, position)));
	vertexElements.push_back(render::VertexElement(render::DataUsage::Custom, render::DtHalf2, offsetof(Vertex, texCoord)));
	T_ASSERT(render::getVertexSize(vertexElements) == sizeof(Vertex));
	m_vertexLayout = renderSystem->createVertexLayout(vertexElements);

	m_vertexBuffer = renderSystem->createBuffer(
		render::BuVertex,
		c_bladeVertexCount * sizeof(Vertex),
		false,
		T_FILE_LINE_W);
	if (!m_vertexBuffer)
		return false;

	Vertex* vertex = static_cast< Vertex* >(m_vertexBuffer->lock());
	if (!vertex)
		return false;

	// Segment rows, root at height 0; quadratic taper towards the tip.
	for (int32_t i = 0; i < c_bladeSegments; ++i)
	{
		const float h = float(i) / c_bladeSegments;
		const float taper = 1.0f - h * h;
		*vertex++ = packVertex(-taper, h, 0.0f, 1.0f - h);
		*vertex++ = packVertex(taper, h, 1.0f, 1.0f - h);
	}

	// Tip vertex.
	*vertex++ = packVertex(0.0f, 1.0f, 0.5f, 0.0f);

	m_vertexBuffer->unlock();

	m_indexBuffer = renderSystem->createBuffer(
		render::BuIndex,
		c_bladeTriangleCount * 3 * sizeof(uint16_t),
		false,
		T_FILE_LINE_W);
	if (!m_indexBuffer)
		return false;

	uint16_t* index = static_cast< uint16_t* >(m_indexBuffer->lock());
	if (!index)
		return false;

	// Both windings so blades are visible from either side.
	for (int32_t i = 0; i < c_bladeSegments - 1; ++i)
	{
		const uint16_t bl = uint16_t(i * 2);
		const uint16_t br = uint16_t(i * 2 + 1);
		const uint16_t tl = uint16_t(i * 2 + 2);
		const uint16_t tr = uint16_t(i * 2 + 3);

		*index++ = bl;
		*index++ = tl;
		*index++ = tr;

		*index++ = bl;
		*index++ = tr;
		*index++ = br;

		*index++ = tr;
		*index++ = tl;
		*index++ = bl;

		*index++ = br;
		*index++ = tr;
		*index++ = bl;
	}

	const uint16_t ll = uint16_t((c_bladeSegments - 1) * 2);
	const uint16_t lr = uint16_t((c_bladeSegments - 1) * 2 + 1);
	const uint16_t tip = uint16_t(c_bladeVertexCount - 1);

	*index++ = ll;
	*index++ = tip;
	*index++ = lr;

	*index++ = lr;
	*index++ = tip;
	*index++ = ll;

	m_indexBuffer->unlock();

	m_renderSystem = renderSystem;
	return true;
}

void GrassComponent::destroy()
{
	m_renderSystem = nullptr;
}

void GrassComponent::setOwner(world::Entity* owner)
{
	TerrainLayerComponent::setOwner(owner);
	m_owner = owner;
}

void GrassComponent::setTransform(const Transform& transform)
{
}

Aabb3 GrassComponent::getBoundingBox() const
{
	return Aabb3();
}

void GrassComponent::update(const world::UpdateParams& update)
{
	TerrainLayerComponent::update(update);
}

bool GrassComponent::updateBladeBuffer()
{
	m_bladeBuffer = nullptr;

	if (!m_bladesCount)
		return true;

	auto terrainComponent = m_owner->getComponent< TerrainComponent >();
	if (!terrainComponent)
		return false;

	const hf::Heightfield* heightfield = terrainComponent->getTerrain()->getHeightfield();

	m_bladeBuffer = m_renderSystem->createBuffer(render::BufferUsage::BuStructured, m_bladesCount * sizeof(BladeData), false, T_FILE_LINE_W);
	if (!m_bladeBuffer)
		return false;

	BladeData* bladeData = (BladeData*)m_bladeBuffer->lock();
	if (!bladeData)
	{
		m_bladeBuffer = nullptr;
		return false;
	}

	for (const Cluster& cluster : m_clusters)
	{
		const auto& grass = m_layerData.m_grass[cluster.grass];
		const int32_t count = scatterCluster(
			heightfield,
			cluster.center,
			m_clusterSize,
			grass.attribute,
			grass.density,
			grass.scale,
			bladeData + cluster.from,
			cluster.to - cluster.from);

		// Should only differ if the heightfield changed since updatePatches; the
		// cluster then render a few blades of stale data until it's rebuilt.
		T_ASSERT(count == cluster.to - cluster.from);
	}

	m_bladeBuffer->unlock();
	return true;
}

void GrassComponent::setup(const world::WorldRenderView& worldRenderView)
{
	if (!m_bladesCount)
		return;

	// Blade data is view independent and constant over time; build it on demand,
	// it's invalidated when the clusters are rebuilt.
	if (m_bladeBuffer == nullptr && !updateBladeBuffer())
		return;

	const Matrix44 view = worldRenderView.getView();

	// Get blade state for current view.
	ViewState& vs = m_viewState[worldRenderView.getIndex()];
	if (vs.orderBuffer == nullptr || vs.orderBuffer->getBufferSize() / sizeof(int32_t) != m_bladesCount)
	{
		vs.drawInstanceCount = 0;
		vs.orderBuffer = m_renderSystem->createBuffer(render::BufferUsage::BuStructured, m_bladesCount * sizeof(int32_t), true, T_FILE_LINE_W);
		if (!vs.orderBuffer)
			return;
	}

	Frustum viewFrustum = worldRenderView.getViewFrustum();
	viewFrustum.setFarZ(Scalar(m_layerData.m_spreadDistance + m_clusterSize));

	vs.drawInstanceCount = 0;

	int32_t* orderPtr = (int32_t*)vs.orderBuffer->lock();
	if (!orderPtr)
		return;

	// Compact indices of visible blades; the blade data itself is already resident
	// and indexed through this list.
	const Scalar clusterSize(m_clusterSize);
	for (const Cluster& cluster : m_clusters)
	{
		if (viewFrustum.inside(view * cluster.center, clusterSize) == Frustum::Result::Outside)
			continue;

		for (int32_t j = cluster.from; j < cluster.to; ++j)
			*orderPtr++ = j;

		vs.drawInstanceCount += cluster.to - cluster.from;
	}

	vs.orderBuffer->unlock();
}

void GrassComponent::build(
	const world::WorldBuildContext& context,
	const world::WorldRenderView& worldRenderView,
	const world::IWorldRenderPass& worldRenderPass)
{
	auto terrainComponent = m_owner->getComponent< TerrainComponent >();
	if (!terrainComponent)
		return;

	if (!m_bladesCount)
		return;

	const auto& terrain = terrainComponent->getTerrain();

	const Matrix44 view = worldRenderView.getView();
	const Matrix44 viewInv = view.inverse();
	const Vector4 eye = viewInv.translation();

	const world::World* world = m_owner->getWorld();
	const world::DisplacementWorldComponent* displacement = world ? world->getComponent< world::DisplacementWorldComponent >() : nullptr;
	if (displacement != nullptr && displacement->getMask() == nullptr)
		displacement = nullptr;

	render::Shader::Permutation perm = worldRenderPass.getPermutation(m_shader);
	world::DisplacementWorldComponent::getPermutation(displacement, m_shader, perm);

	auto sp = m_shader->getProgram(perm);
	if (!sp)
		return;

	if (!m_bladeBuffer)
		return;

	ViewState& vs = m_viewState[worldRenderView.getIndex()];
	if (!vs.orderBuffer || vs.drawInstanceCount <= 0)
		return;

	render::RenderContext* renderContext = context.getRenderContext();

	auto renderBlock = renderContext->allocNamed< render::IndexedInstancingRenderBlock >(L"Grass");
	renderBlock->distance = 10000.0f;
	renderBlock->program = sp.program;
	renderBlock->programParams = renderContext->alloc< render::ProgramParameters >();
	renderBlock->indexBuffer = m_indexBuffer->getBufferView();
	renderBlock->indexType = render::IndexType::UInt16;
	renderBlock->vertexBuffer = m_vertexBuffer->getBufferView();
	renderBlock->vertexLayout = m_vertexLayout;
	renderBlock->primitive = render::PrimitiveType::Triangles;
	renderBlock->offset = 0;
	renderBlock->count = c_bladeTriangleCount;
	renderBlock->instanceCount = vs.drawInstanceCount;

	renderBlock->programParams->beginParameters(renderContext);
	worldRenderPass.setProgramParameters(renderBlock->programParams);
	renderBlock->programParams->setTextureParameter(s_handleTerrain_Normals, terrain->getNormalMap());
	renderBlock->programParams->setTextureParameter(s_handleTerrain_Heightfield, terrain->getHeightMap());
	renderBlock->programParams->setTextureParameter(s_handleTerrain_SurfaceAlbedo, terrainComponent->getSurfaceCache()->getBaseTexture());
	renderBlock->programParams->setVectorParameter(s_handleTerrain_WorldExtent, terrain->getHeightfield()->getWorldExtent());
	renderBlock->programParams->setVectorParameter(s_handleGrass_Eye, eye);
	renderBlock->programParams->setFloatParameter(s_handleGrass_MaxDistance, m_layerData.m_spreadDistance + m_clusterSize);
	renderBlock->programParams->setBufferViewParameter(s_handleGrass_Blades, m_bladeBuffer->getBufferView());
	renderBlock->programParams->setBufferViewParameter(s_handleGrass_Order, vs.orderBuffer->getBufferView());
	world::DisplacementWorldComponent::setSharedParameters(displacement, renderBlock->programParams);
	renderBlock->programParams->endParameters(renderContext);

	renderContext->draw(
		sp.priority,
		renderBlock);
}

void GrassComponent::updatePatches()
{
	m_clusters.resize(0);
	m_bladesCount = 0;

	// Blade buffer is built from the clusters; invalidate it and let setup rebuild
	// it on the next frame.
	m_bladeBuffer = nullptr;

	auto terrainComponent = m_owner->getComponent< TerrainComponent >();
	if (!terrainComponent)
		return;

	const hf::Heightfield* heightfield = terrainComponent->getTerrain()->getHeightfield();

	const int32_t size = heightfield->getSize();
	const Vector4 extentPerGrid = heightfield->getWorldExtent() / Scalar(float(size));

	m_clusterSize = (c_clusterGridSize / 2.0f) * max< float >(extentPerGrid.x(), extentPerGrid.z());

	// Create clusters; a cluster only determine which blades are scattered and culled
	// together, the density within it follow the attribute channel per blade.
	for (int32_t z = 0; z < size; z += c_clusterGridSize)
	{
		for (int32_t x = 0; x < size; x += c_clusterGridSize)
		{
			float wx, wz;
			heightfield->gridToWorld(x + c_clusterGridSize / 2, z + c_clusterGridSize / 2, wx, wz);

			const float wy = heightfield->getWorldHeight(wx, wz);
			const Vector4 center(wx, wy, wz, 1.0f);

			for (uint32_t i = 0; i < m_layerData.m_grass.size(); ++i)
			{
				const auto& grass = m_layerData.m_grass[i];

				// Density is filtered, thus a one cell border is included since a cluster
				// next to a painted region also carry a fringe of grass.
				if (!anyAttributeDensity(heightfield, x - 1, z - 1, c_clusterGridSize + 2, grass.attribute))
					continue;

				const int32_t density = scatterCluster(
					heightfield,
					center,
					m_clusterSize,
					grass.attribute,
					grass.density,
					grass.scale,
					nullptr,
					0);
				if (density <= 0)
					continue;

				Cluster c;
				c.center = center;
				c.grass = (int32_t)i;
				c.from = m_bladesCount;
				c.to = c.from + density;
				m_clusters.push_back(c);

				m_bladesCount = c.to;
			}
		}
	}
}

}
