#include "test.h"
#include "geometry/halfedge.h"

static void expect_flip(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.flip_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check if returned edge is the same edge
		if (ret != edge) {
			throw Test::error("Did not return the same edge!");
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("flip_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 1-4

After mesh:
0--1\
|\   \
| \---2
|    /
3--4/
*/
Test test_a2_l1_flip_edge_basic_simple("a2.l1.flip_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 2}, 
		{0, 2, 1}
	});

	expect_flip(mesh, edge, after);
});

/*
BASIC CASE

Initial mesh:

    1
   /| \
  / |  \
0   |   3
  \ |   /
	 \|  /
 	 	2 

Flip Edge on Edge: 1-2

After mesh:

    1
   /  \
  /    \
0-------3
  \     /
	 \   /
 	 	2 


*/
Test test_a2_l1_flip_edge_basic_triangle("a2.l1.flip_edge.basic.triangle", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		             Vec3(-1.0f, 1.1f, 0.0f),
		Vec3(-2.1f, 0.3f, 0.0f), Vec3(1.0f, 0.2f, 0.0f),
		            Vec3(-1.1f, -1.1f, 0.0f)
	}, {
		{0, 2, 1},
		{1, 2, 3}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		             Vec3(-1.0f, 1.1f, 0.0f),
		Vec3(-2.1f, 0.3f, 0.0f), Vec3(1.0f, 0.2f, 0.0f),
		            Vec3(-1.1f, -1.1f, 0.0f)
	}, {
		{0, 3, 1}, 
		{0, 2, 3}
	});

	expect_flip(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 3-4

After mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/
*/
Test test_a2_l1_flip_edge_edge_boundary("a2.l1.flip_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work at the boundary.");
	}
});

/*
Edge case

Initial: 

0---1---2
|   |   |
|   |   |
|   3   |
|   |   |
4---5---6

Flip edge 1--6

Wrong result:

0---1---2
|\       |
| \      |
|  \  3  |
|   \|   |
4---5----6

Correct result (don't flip):

0---1---2
|   |   |
|   |   |
|   3   |
|   |   |
4---5---6
*/
Test test_a2_l1_flip_edge_hard_1("a2.l1.flip_edge_hard_1", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
	Vec3(-1.0f, 1.1f, 0.0f), Vec3(0.5f, 1.0f, 0.0f), Vec3(1.1f, 0.9f, 0.0f),
													 Vec3(0.6f, 0.1f, 0.0f),										
	Vec3(-1.5f,-0.7f, 0.0f), Vec3(0.1f, -0.8f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 4, 5, 3, 1},
		{1, 3, 5, 6, 2}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not result in an invalid mesh.");
	}
});

/*
Edge case

Initial: 

0---1---2
|   |   |
|   3   |
|   |   |
|   4   |
|   |   |
5---6---7

Flip edge 3--4

Wrong result:

0---1---2
|\  \    |
| \  3   |
|  \     |
|   \ 4  |
|    |   |
5----6---7

Correct result (don't flip):

0---1---2
|   |   |
|   3   |
|   |   |
|   4   |
|   |   |
5---6---7
*/
Test test_a2_l1_flip_edge_hard_2("a2.l1.flip_edge_hard_2", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
	Vec3(-1.0f, 1.1f, 0.0f), Vec3(0.5f, 1.0f, 0.0f), Vec3(1.1f, 0.9f, 0.0f),
													 Vec3(0.6f, 0.4f, 0.0f),
													 Vec3(0.5f, -0.3f, 0.0f),										
	Vec3(-1.5f,-0.7f, 0.0f), Vec3(0.1f, -0.8f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 5, 6, 4, 3, 1},
		{1, 3, 4, 6, 7, 2}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not result in an invalid mesh.");
	}
});