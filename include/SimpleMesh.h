#pragma once
#include "Definitions.h"
#include "VirtualSensor.h"

class SimpleMesh
{
public:
	SimpleMesh();
	SimpleMesh(VirtualSensor &sensor, const Matrix4f &cameraPose, float edgeThreshold = 0.01f);
	Vector3f inverseProject(float x, float y, float depth, float fX, float fY, float cX, float cY);

	void clear();

	unsigned int addVertex(Vertex &vertex);
	unsigned int addFace(unsigned int idx0, unsigned int idx1, unsigned int idx2);
	std::vector<Vertex> &getVertices();
	const std::vector<Vertex> &getVertices() const;
	std::vector<Triangle> &getTriangles();
	const std::vector<Triangle> &getTriangles() const;

	void transform(const Matrix4f &transformation);

	bool loadMesh(const std::string &filename);

	bool writeMesh(const std::string &filename);

	static SimpleMesh joinMeshes(const SimpleMesh &mesh1, const SimpleMesh &mesh2, Matrix4f pose1to2 = Matrix4f::Identity());

	static SimpleMesh sphere(Vector3f center, float scale = 1.f, Vector4uc color = {0, 0, 255, 255});

	static SimpleMesh camera(const Matrix4f &cameraPose, float scale = 1.f, Vector4uc color = {255, 0, 0, 255});

	static SimpleMesh cylinder(const Vector3f &p0, const Vector3f &p1, float radius, unsigned stacks, unsigned slices, const Vector4uc color = Vector4uc{0, 0, 255, 255});

private:
	std::vector<Vertex> m_vertices;
	std::vector<Triangle> m_triangles;
	static Matrix3f face(const Vector3f &vA, const Vector3f &vB);
};
