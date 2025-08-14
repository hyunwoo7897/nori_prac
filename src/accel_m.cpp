/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/accel.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    m_meshes.push_back(mesh);
    // 전체 bbox 갱신
    if (m_meshes.size() == 1)
        m_bbox = mesh->getBoundingBox();
    else
        m_bbox.expandBy(mesh->getBoundingBox());
}

void Accel::build() {
    // 1. 전체 삼각형 인덱스와 정점 배열을 합침
    std::vector<uint32_t> allIndices;
    std::vector<Mesh*> meshForFace; // 각 삼각형이 어느 mesh에 속하는지 저장
    MatrixXu allFaces(3, 0);
    MatrixXf allVerts(3, 0);

    // 각 mesh의 정점 오프셋을 관리
    uint32_t vertOffset = 0;
    for (Mesh* mesh : m_meshes) {
        const MatrixXu& F = mesh->getIndices();
        const MatrixXf& V = mesh->getVertexPositions();

        // 정점 추가
        int prevCols = allVerts.cols();
        allVerts.conservativeResize(3, prevCols + V.cols());
        allVerts.block(0, prevCols, 3, V.cols()) = V;

        // face(삼각형) 인덱스 추가 (정점 오프셋 반영)
        int prevFaceCols = allFaces.cols();
        allFaces.conservativeResize(3, prevFaceCols + F.cols());
        for (int i = 0; i < F.cols(); ++i) {
            allFaces(0, prevFaceCols + i) = F(0, i) + vertOffset;
            allFaces(1, prevFaceCols + i) = F(1, i) + vertOffset;
            allFaces(2, prevFaceCols + i) = F(2, i) + vertOffset;
            meshForFace.push_back(mesh);
        }
        vertOffset += V.cols();
    }

    // 2. 전체 bbox 계산
    m_bbox.reset();
    for (Mesh* mesh : m_meshes)
        m_bbox.expandBy(mesh->getBoundingBox());

    // 3. scene 전체 Octree 생성
    m_octree = std::make_unique<Octree>(m_bbox, allFaces, allVerts);

    // 4. meshForFace를 멤버로 저장 (rayIntersect에서 사용)
    m_meshForFace = meshForFace;
}


bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;
    Ray3f ray(ray_);
    uint32_t f = (uint32_t)-1;
    float closestT = std::numeric_limits<float>::infinity();

    Node* leafNodes[256];
    size_t leafCount = m_octree->traverseClosest(m_octree->root, ray, leafNodes, 256);
    for (size_t i = 0; i < leafCount; ++i) {
        Node* leaf = leafNodes[i];
        for (uint32_t idx : leaf->triangleIndices) {
            float u, v, t;
            // 어떤 mesh의 몇 번째 삼각형인지 알아야 하므로
            Mesh* mesh = m_meshForFace[idx];
            if (mesh->rayIntersect(idx, ray, u, v, t)) {
                if (shadowRay)
                    return true;
                if (!foundIntersection || t < closestT) {
                    closestT = t;
                    ray.maxt = its.t = t;
                    its.uv = Point2f(u, v);
                    its.mesh = mesh;
                    f = idx;
                    foundIntersection = true;
                }
            }
        }
        if (foundIntersection)
            break;
    }

    if (foundIntersection) {
        // 기존과 동일하게 its 보간 등 처리
        Vector3f bary;
        bary << 1 - its.uv.sum(), its.uv;
        const Mesh* mesh = its.mesh;
        const MatrixXf& V = mesh->getVertexPositions();
        const MatrixXf& N = mesh->getVertexNormals();
        const MatrixXf& UV = mesh->getVertexTexCoords();
        const MatrixXu& F = mesh->getIndices();

        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);
        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                     bary.y() * UV.col(idx1) +
                     bary.z() * UV.col(idx2);
        its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());
        if (N.size() > 0) {
            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}


Node::Node(const BoundingBox3f& box, const std::vector<uint32_t>& tris)
    : boundingBox(box), triangleIndices(tris) {}

Node::~Node() {
    for (Node* child : children)
        delete child;
}

bool Node::isLeaf() const {
    return children.empty();
}

Octree::Octree(const BoundingBox3f& box, const MatrixXu& faces, const MatrixXf& verts, int maxDepth_ = 7, int minTriangles_ = 10)
    : maxDepth(maxDepth_), minTriangles(minTriangles_), m_F(faces), m_V(verts) {
    std::vector<uint32_t> allTris(m_F.cols());
    for (uint32_t i = 0; i < m_F.cols(); ++i)
        allTris[i] = i;
    root = build(box, allTris, 0);
}

Octree::~Octree() {
    delete root;
}

Node* Octree::build(const BoundingBox3f& box, const std::vector<uint32_t>& tris, int depth) {
    
    if (tris.empty())
        return nullptr;
    else if (depth > maxDepth)
        return new Node(box, tris);

    if (tris.size() <= minTriangles)
        return new Node(box, tris);

    std::vector<std::vector<uint32_t>> childTris(8);
    Vector3f mid = box.getCenter();

    for (uint32_t idx : tris) {
        auto fidx = m_F.col(idx); 
        Vector3f v0 = m_V.col(fidx[0]);
        Vector3f v1 = m_V.col(fidx[1]);
        Vector3f v2 = m_V.col(fidx[2]);
        BoundingBox3f triBox;
        triBox.reset();
        triBox.expandBy(v0);
        triBox.expandBy(v1);
        triBox.expandBy(v2);
        for (int j = 0; j < 8; ++j) {
            BoundingBox3f childBox = getChildBoundingBox(box, mid, j);

            if (childBox.overlaps(triBox)) {
                childTris[j].push_back(idx);
            }
        }
    }

    Node* node = new Node(box, tris);
    for (int i = 0; i < 8; ++i) {
        Node* child = build(getChildBoundingBox(box, mid, i), childTris[i], depth + 1);
        if (child){
            node->children.push_back(child);
        }   
    }
    if(!node->children.empty()) {
        node->triangleIndices.clear(); 
    }
    return node;
}


BoundingBox3f Octree::getChildBoundingBox(const BoundingBox3f& box, const Vector3f& mid, int idx) const {
    Vector3f cmin, cmax;
    for (int i = 0; i < 3; ++i) {
        cmin[i] = (idx & (1 << i)) ? mid[i] : box.min[i];
        cmax[i] = (idx & (1 << i)) ? box.max[i] : mid[i];
    }
    return BoundingBox3f(cmin, cmax);
}


struct LeafHit {
    Node* node;
    float tNear;
};

int Octree::traverseClosest(Node* node, const Ray3f& ray, Node** outLeafNodes, int maxLeafCount) const {
    LeafHit leafHits[256]; // leaf node 최대 개수만큼 (필요시 크기 조정)
    int leafCount = 0;

    // 재귀적으로 leaf node와 tNear를 배열에 저장
    std::function<void(Node*)> traverse = [&](Node* n) {
        if (!n || leafCount >= maxLeafCount) return;
        float tNear, tFar;
        if (!n->boundingBox.rayIntersect(ray, tNear, tFar) || tFar < tNear || tFar < 0)
            return;
        if (n->isLeaf()) {
            leafHits[leafCount].node = n;
            leafHits[leafCount].tNear = tNear;
            ++leafCount;
        } else {
            for (Node* child : n->children)
                traverse(child);
        }
    };

    traverse(root);

    // tNear 기준으로 정렬
    std::sort(leafHits, leafHits + leafCount, [](const LeafHit& a, const LeafHit& b) {
        return a.tNear < b.tNear;
    });

    // 정렬된 결과를 outLeafNodes에 복사
    for (int i = 0; i < leafCount; ++i)
        outLeafNodes[i] = leafHits[i].node;

    return leafCount; // 저장된 leaf node 개수 반환
}
NORI_NAMESPACE_END

