
#include "halfedge.h"

#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge

	std::optional<VertexRef> vopt = bisect_edge(e);
	if (vopt == std::nullopt) {
		return vopt;
	}

	VertexRef v = vopt.value();

	HalfedgeRef h1 = v->halfedge;
	HalfedgeRef t1 = h1->twin;
	HalfedgeRef h2 = t1->next->twin;
	HalfedgeRef t2 = t1->next;
	HalfedgeRef hp = t2->next->twin;
	HalfedgeRef tn = hp->twin;
	HalfedgeRef hn = h1->next;
	HalfedgeRef tp = hn->twin;

	VertexRef v1 = t1->vertex;
	VertexRef v2 = h2->vertex;
	VertexRef v3 = tp->vertex;
	VertexRef v4 = hp->vertex;

	FaceRef f1 = t1->face;
	FaceRef f2 = h1->face;

	if (f1->boundary && f2->boundary) {
		return std::nullopt;
	}

	if (!f1->boundary) {
		// std::cout << "\nHERE\n";
		FaceRef f3 = emplace_face();
		EdgeRef el = emplace_edge();
		HalfedgeRef hl = emplace_halfedge();
		HalfedgeRef tl = emplace_halfedge();

		f3->halfedge = hl;
		el->halfedge = hl;

		hl->vertex = v4;
		hl->twin = tl;
		hl->edge = el;
		hl->face = f3;
		hl->next = t2;

		tl->vertex = v;
		tl->twin = hl;
		tl->edge = el;
		tl->face = f1;
		tl->next = tn->next;

		t2->face = f3;
		tn->face = f3;
		t1->next = tl;
		tn->next = hl;

		f1->halfedge = t1;
	}	

	if (!f2->boundary) {
		// std::cout << "here\n";
		FaceRef f4 = emplace_face();
		EdgeRef er = emplace_edge();
		HalfedgeRef hr = emplace_halfedge();
		HalfedgeRef tr = emplace_halfedge();

		f4->halfedge = tr;
		er->halfedge = tr;

		hr->vertex = v;
		hr->twin = tr;
		hr->edge = er;
		hr->face = f2;
		hr->next = hn->next;

		tr->vertex = v3;
		tr->twin = hr;
		tr->edge = er;
		tr->face = f4;
		tr->next = h1;

		h1->face = f4;
		hn->face = f4;
		h2->next = hr;
		hn->next = tr;

		f2->halfedge = h2;
	}

	std::cout << describe() << std::endl;
  return v;
}



/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	int n = f->degree();

	std::vector<HalfedgeRef> oh; 
	std::vector<HalfedgeRef> ot; 
	std::vector<VertexRef>   ov; 
	std::vector<EdgeRef>     oe; 

	HalfedgeRef h = f->halfedge, c = h;
	do {
		oh.push_back(c);
		ot.push_back(c->twin);
		ov.push_back(c->vertex);
		oe.push_back(c->edge);
		c = c->next;
	} while (c != h);

	std::vector<VertexRef>   bv(n);
	std::vector<FaceRef>     fin(n);
	std::vector<EdgeRef>     ec(n), be(n);
	std::vector<HalfedgeRef> ch(n), ct(n), bt(n), bh(n);

	for (size_t i = 0; i < n; i++) {
		bv[i] = emplace_vertex(); 
		bv[i]->position = ov[i]->position;
		fin[i] = emplace_face();
		ec[i] = emplace_edge();
		be[i] = emplace_edge();
		bh[i] = emplace_halfedge();
		bt[i] = emplace_halfedge();
		ch[i] = emplace_halfedge();
		ct[i] = emplace_halfedge();
	}

	for (int i = 0; i < n; i++) {
		int ni = (i + 1) % n;
		int pi = (i - 1 + n) % n;

		ch[i]->vertex = ov[ni];
		ch[i]->edge = ec[i];
		ch[i]->twin = ct[i];
		ch[i]->next = bt[i];
		ch[i]->face = fin[i];

		ct[i]->vertex = bv[ni];
		ct[i]->edge = ec[i];
		ct[i]->twin = ch[i];
		ct[i]->next = oh[ni];
		ct[i]->face = fin[ni];

		bh[i]->vertex = bv[i];
		bh[i]->edge = be[i];
		bh[i]->twin = bt[i];
		bh[i]->next = bh[ni];
		bh[i]->face = f;

		bt[i]->vertex = bv[ni];
		bt[i]->edge = be[i];
		bt[i]->twin = bh[i];
		bt[i]->next = ct[pi];
		bt[i]->face = fin[i];

		ec[i]->halfedge = ch[i];
		be[i]->halfedge = bt[i];
		fin[i]->halfedge = ch[i];
		bv[i]->halfedge = bh[i];

		oh[i]->next = ch[i];
		oh[i]->face = fin[i];
	}

	f->halfedge = bh[0];

	std::cout << describe() << std::endl;
	return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge

	// collect
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	// edge case: when either vertices incident to the edge has degree <= 2, reject
	VertexRef v1 = h->next->vertex, v2 = t->next->vertex;
  if (e->on_boundary() || v1->degree() <= 2 || v2->degree() <= 2) {
		return std::nullopt;
	}

	VertexRef v3 = h->next->next->vertex;
	VertexRef v4 = t->next->next->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	HalfedgeRef hn = h->next;
	HalfedgeRef tn = t->next;

	HalfedgeRef hp = h->next;
	while (hp->next != h) { hp = hp->next; }
	HalfedgeRef tp = t->next;
	while (tp->next != t) { tp = tp->next; }

	// connect
	v1->halfedge = hn;
	v2->halfedge = tn;
	
	t->vertex = v3;
	h->vertex = v4;

	h->next = hn->next;
	t->next = tn->next;
	tp->next = hn;
	hp->next = tn;
	hn->next = t;
	tn->next = h;

	h->face = f1;
	hn->face = f2;

	t->face = f2;
	tn->face = f1;

	f1->halfedge = h;
	f2->halfedge = t;
	
	return e;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge
	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

	HalfedgeRef h = e->halfedge;
	VertexRef v1 = h->vertex;
	VertexRef v2 = h->twin->vertex;

	// refer to: https://hhoppe.com/meshopt.pdf

	// 1. for any v3 adjacent to both v1 and v2, {v1, v2, v3} must form a face
	std::unordered_set<VertexRef> v1_neighbours{};
	HalfedgeRef h_temp = v1->halfedge;
	do {
		v1_neighbours.insert(h_temp->twin->vertex);
		h_temp = h_temp->twin->next;
	} while (h_temp != v1->halfedge);

	h_temp = v2->halfedge;
	do {
		VertexRef v3 = h_temp->twin->vertex;
		if (v1_neighbours.find(v3) != v1_neighbours.end()) {
			// check if {v1, v2, v3} form a face
			bool forms_face = false;
			HalfedgeRef h_v3 = v3->halfedge;
			do {
				FaceRef f = h_v3->face;
				if (!f->boundary) {
					HalfedgeRef h_face = f->halfedge;
					std::unordered_set<VertexRef> face_vertices;
					do {
						face_vertices.insert(h_face->vertex);
						h_face = h_face->next;
					} while (h_face != f->halfedge);
					
					if (face_vertices.count(v1) && face_vertices.count(v2) && face_vertices.count(v3)) {
						forms_face = true;
						break;
					}
				}
				h_v3 = h_v3->twin->next;
			} while (h_v3 != v3->halfedge);
			
			if (!forms_face) {
					return std::nullopt;
			}
		}
		h_temp = h_temp->twin->next;
	} while (h_temp != v2->halfedge);

	// 2. if v1 and v2 are boundary vertices, so must v1--v2 be
	if (v1->on_boundary() && v2->on_boundary() && !e->on_boundary()) {
		return std::nullopt;
	}

	// 3. If v1 and v2 are non-boundary, require #vertices >= 4; If either of them is, require #vertices >= 3
	size_t vcnt = this->vertices.size();
	if (!v1->on_boundary() && !v2->on_boundary() && vcnt <= 3
			|| (v1->on_boundary() || v2->on_boundary()) && vcnt <= 2) {
		return std::nullopt;
	}
		

	/* -------- FROM NOW ON ASSUME EDGE COLLAPSING IS POSSIBLE ---------- */


	FaceRef f1 = h->face;
	FaceRef f2 = h->twin->face;
	
	// don't worry about non-triangular faces for now
	if (f1->degree() != 3 || f2->degree() != 3) {
		return std::nullopt;
	}
	
	// collapse enclosing cycle in which e->halfedge sits
	if (!f1->boundary) {

	}

	// collapse enclosing cycle in which e->halfedge sits (or e->halfedge->twin if the above part was also executed)
	if (!f2->boundary) {

	}

  return std::nullopt;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move
	
	HalfedgeRef h = face->halfedge;
	Vec3 centroid = face->center();

	HalfedgeRef h_current = h;
	do {
		VertexRef v = h_current->vertex;
		Vec3 op = v->position;
		v->position = op + shrink * (centroid - op) + move;
		
		h_current = h_current->next;
	} while (h_current != h);
}

