#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Task 1.

    std::vector<Vector2D> result;

    for(int i=0; i<points.size()-1; i++){
      Vector2D new_point;
      new_point = points[i] * (1-t) + points[i+1] * t;
      result.push_back(new_point);
    }

    return result;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    std::vector<Vector3D> result;

    for(int i=0; i<points.size()-1; i++){
      Vector3D new_point;
      new_point = points[i] * (1-t) + points[i+1] * t;
      result.push_back(new_point);
    }

    return result;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    std::vector<Vector3D> temp;
    
    temp = points;
    while(temp.size() != 1){
      temp = evaluateStep(temp,t);
    }
    return temp[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Task 2.
    vector<Vector3D> col;
    for(auto row : controlPoints){
      Vector3D temp = evaluate1D(row, u);
      col.push_back(temp);
    }

    Vector3D result;
    result = evaluate1D(col,v);


    return result;
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Task 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D normal( 0., 0., 0. );

    HalfedgeCIter h = halfedge();

    do
    {
      if(! h->face()->isBoundary()){
        Vector3D p0 = h->vertex()->position;
        Vector3D p1 = h->next()->vertex()->position;
        Vector3D p2 = h->next()->next()->vertex()->position;

        double area = cross(p1-p0, p2-p0).norm() / 2;
        normal += area * h->face()->normal();
        
      }
      h = h->twin()->next();
    }
    while( h != halfedge() );

    return normal.unit();

  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Task 4.
    // This method should flip the given edge and return an iterator to the flipped edge.

    if (e0->isBoundary()){
      return e0;
    }

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    FaceIter f1 = h3->face();
    FaceIter f2 = h0->face();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();


    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();


    /*void setNeighbors( HalfedgeIter next,
                            HalfedgeIter twin,
                            VertexIter vertex,
                            EdgeIter edge,
                            FaceIter face )*/

    h0->setNeighbors(h1,h3,v3,e0,f2);
    h1->setNeighbors(h2,h7,v2,e2,f2);
    h2->setNeighbors(h0,h8,v0,e3,f2);
    h3->setNeighbors(h4,h0,v2,e0,f1);
    h4->setNeighbors(h5,h9,v3,e4,f1);
    h5->setNeighbors(h3,h6,v1,e1,f1);
    h6->setNeighbors(h6->next(),h5,v2,e1,h6->face());
    h7->setNeighbors(h7->next(),h1,v0,e2,h7->face());
    h8->setNeighbors(h8->next(),h2,v3,e3,h8->face());
    h9->setNeighbors(h9->next(),h4,v1,e4,h9->face());

    e0->halfedge() = h0;
    e1->halfedge() = h5;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;

    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h1;
    v3->halfedge() = h4;

    f1->halfedge() = h3;
    f2->halfedge() = h0;

    // cout << (h0->next() == h1) << endl;
    // cout << (h1->next() == h2) << endl;
    // cout << (h2->next() == h0) << endl;
    // cout << (h3->next() == h4) << endl;
    // cout << (h4->next() == h5) << endl;
    // cout << (h5->next() == h3) << endl;

    // cout << "h0 " << (long long)&h0 << endl;
    // cout << "h0 next " << (long long)&h0->next() << endl;
    // cout << "h1 " << (long long)&h1 << endl;
    // cout << "h1 next " << (long long)&h1->next() << endl;
    // cout << "h2 " << (long long)&h2 << endl;
    // cout << "h2 next " << (long long)&h2->next() << endl;

    // cout << "h3 " << (long long)&h3 << endl;
    // cout << "h3 next " << (long long)&h3->next() << endl;
    // cout << "h4 " << (long long)&h4 << endl;
    // cout << "h4 next " << (long long)&h4->next() << endl;
    // cout << "h5 " << (long long)&h5 << endl;
    // cout << "h5 next " << (long long)&h5->next() << endl;

    

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Task 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    if (e0->isBoundary()){
      return VertexIter();
    }

    

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();


    FaceIter f1 = h3->face();
    FaceIter f2 = h0->face();

    FaceIter f0 = newFace();
    FaceIter f3 = newFace();


    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    VertexIter v4 = newVertex();
    v4->position = (v1->position + v0->position) / 2;

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

    //mark new edge and vertex as new
 
    e6->isNew = true;
    e7->isNew = true;

    v4->isNew = true;


    h0->setNeighbors(h1,h3,v4,e0,f0);
    h1->setNeighbors(h12,h6,v1,e1,f0);
    h2->setNeighbors(h15,h7,v2,e2,f3);
    h3->setNeighbors(h10,h0,v1,e0,f1);
    h4->setNeighbors(h11,h8,v0,e3,f2);
    h5->setNeighbors(h3,h9,v3,e4,f1);
    h6->setNeighbors(h6->next(),h1,v2,e1,h6->face());
    h7->setNeighbors(h7->next(),h2,v0,e2,h7->face());
    h8->setNeighbors(h8->next(),h4,v3,e3,h8->face());
    h9->setNeighbors(h9->next(),h5,v1,e4,h9->face());
    h10->setNeighbors(h5,h11,v4,e6,f1);
    h11->setNeighbors(h14,h10,v3,e6,f2);
    h12->setNeighbors(h0,h13,v2,e7,f0);
    h13->setNeighbors(h2,h12,v4,e7,f3);
    h14->setNeighbors(h4,h15,v4,e5,f2);
    h15->setNeighbors(h13,h14,v0,e5,f3);


    v0->halfedge() = h7;
    v1->halfedge() = h9;
    v2->halfedge() = h6;
    v3->halfedge() = h8;
    v4->halfedge() = h0;

    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h2;
    e3->halfedge() = h4;
    e4->halfedge() = h5;
    e5->halfedge() = h15;
    e6->halfedge() = h10;
    e7->halfedge() = h12;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h4;
    f3->halfedge() = h2;

    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Task 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.


    EdgeIter edge_temp = mesh.edgesBegin();
    EdgeIter edge_end = mesh.edgesEnd();

    VertexIter vertex_temp = mesh.verticesBegin();
    VertexIter vertex_end = mesh.verticesEnd();

    //compute the updated position of old vertex
    while(vertex_temp != vertex_end){
      Vector3D new_pos(0.,0.,0.);
      HalfedgeIter h = vertex_temp->halfedge();
      size_t n = vertex_temp->degree();
      double u;
      if(n == 3) u = 3./16.; else u = 3./(8.*n); 

      new_pos += (1-n*u) * vertex_temp->position;

      do
      {
        h = h->twin();
        new_pos += u * h->vertex()->position;
        h = h->next();
      }
      while( h != vertex_temp->halfedge() );
      // cout << "checkpoint1" <<endl;

      // cout << "pos = " << vertex_temp->position.x << " " << vertex_temp->position.y<<" "<< vertex_temp->position.z<<endl;
      // cout << "new_pos = " << new_pos.x << " " << new_pos.y<<" "<< new_pos.z<<endl;

      vertex_temp->newPosition = new_pos;

      vertex_temp++;

    }


    //compute the updated position of new vertex
    while(edge_temp != edge_end){
      HalfedgeIter h = edge_temp->halfedge();
      Vector3D A = h->vertex()->position;
      Vector3D B = h->next()->vertex()->position;
      Vector3D C = h->next()->next()->vertex()->position;
      Vector3D D = h->twin()->next()->next()->vertex()->position;

      Vector3D new_pos = (3./8.) * (A+B) + (1./8.) * (C+D);
      edge_temp->newPosition = new_pos;

      // cout << "checkpoint2" << new_pos.x << endl;

      edge_temp->isNew = false;
      edge_temp++; 

    }

    

    //update the mesh
    edge_temp = mesh.edgesBegin();
    int edge_num = mesh.nEdges();
    while(edge_num > 0){

      if(!edge_temp->isBoundary()){
        VertexIter new_v = mesh.splitEdge(edge_temp);

        //update position of new vertex using the data stored in edge
        new_v->position = edge_temp->newPosition;

      }
      
      // cout << "checkpoint3 " << &edge_temp << " "<<&edge_end << endl;
      

      edge_num--;
      edge_temp++; 

    }

    

    //update the position of old vertex
    vertex_temp = mesh.verticesBegin();
    while(vertex_temp != vertex_end){
      if(!vertex_temp->isNew){
        vertex_temp->position = vertex_temp->newPosition;
      }
      
      // cout << "checkpoint4" <<endl;
      vertex_temp++;
    }

    //flip the edge that connect an old vertex and a new vertex
    edge_temp = mesh.edgesBegin();
    while(edge_temp != edge_end){

      if(edge_temp->isNew){
        HalfedgeIter h1 = edge_temp->halfedge();
        HalfedgeIter h2 = h1->twin();

        if (h1->vertex()->isNew != h2->vertex()->isNew){
          mesh.flipEdge(edge_temp);
        }
      }

      
      edge_temp++; 

    }

  }
}
