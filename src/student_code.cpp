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
    // TODO Part 1.
    vector<Vector2D> returnPoints;

    for (int i = 1; i < points.size(); ++i) {
      Vector2D lerp = (1-t) * points[i-1] + t * points[i];
      returnPoints.push_back(lerp);
    }
    return returnPoints;
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
    // TODO Part 2.
    std::vector<Vector3D> returnPoints;

    for (int i = 1; i < points.size(); ++i) {
      Vector3D lerp = (1-t) * points[i-1] + t * points[i];
      returnPoints.push_back(lerp);
    }

    return returnPoints;
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
    // TODO Part 2.
    vector<Vector3D> returnPoints;

    returnPoints = evaluateStep(points, t);

    while (returnPoints.size() > 1)
      returnPoints = evaluateStep(returnPoints,t);
    return returnPoints[0];
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
    vector<Vector3D> points1D;
    for (int i = 0; i < controlPoints.size(); ++i) {
      points1D.push_back(evaluate1D(controlPoints[i], u));
    }

    return evaluate1D(points1D, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    Vector3D ans = Vector3D(0,0,0);
    HalfedgeCIter h = halfedge();
    do {
      Vector3D A = h->vertex()->position;
      Vector3D B = h->next()->vertex()->position;
      Vector3D C = h->next()->next()->vertex()->position;

      ans = ans + cross(B-A, C-A);

      HalfedgeCIter h_twin = h -> twin();
      VertexCIter v = h_twin -> vertex();
      h = h_twin -> next();
    } while (h!= halfedge());
    return ans.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if(e0->isBoundary())
        return e0;

    HalfedgeIter halfedgeIter_BC = e0->halfedge();
    HalfedgeIter halfedgeIter_CB = halfedgeIter_BC->twin();

    HalfedgeIter halfedgeIter_CA = halfedgeIter_BC->next();
    HalfedgeIter halfedgeIter_AB = halfedgeIter_CA->next();

    HalfedgeIter halfedgeIter_BD = halfedgeIter_CB->next();
    HalfedgeIter halfedgeIter_DC = halfedgeIter_BD->next();

    FaceIter faceIter_1 = halfedgeIter_BC->face();
    FaceIter faceIter_2 = halfedgeIter_CB->face();

    VertexIter vertexIterB = halfedgeIter_BC->vertex();
    VertexIter vertexIterC = halfedgeIter_CB->vertex();
    VertexIter vertexIterA = halfedgeIter_AB->vertex();
    VertexIter vertexIterD = halfedgeIter_DC->vertex();


    faceIter_1->halfedge() = halfedgeIter_BC;
    faceIter_2->halfedge() = halfedgeIter_CB;

    vertexIterB->halfedge() = halfedgeIter_BD;
    vertexIterC->halfedge() = halfedgeIter_CA;

    halfedgeIter_BC->next() = halfedgeIter_DC;
    halfedgeIter_DC->next() = halfedgeIter_CA;
    halfedgeIter_CA->next() = halfedgeIter_BC;

    halfedgeIter_CB->next() = halfedgeIter_AB;
    halfedgeIter_AB->next() = halfedgeIter_BD;
    halfedgeIter_BD->next() = halfedgeIter_CB;

    halfedgeIter_BC->vertex() = vertexIterA;
    halfedgeIter_CB->vertex() = vertexIterD;

    halfedgeIter_DC->face() = faceIter_1;
    halfedgeIter_AB->face() = faceIter_2;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    HalfedgeIter halfedgeIter_BC = e0->halfedge();
    HalfedgeIter halfedgeIter_CB = halfedgeIter_BC->twin();

    VertexIter vertexIter_B = halfedgeIter_BC->vertex();
    VertexIter vertexIter_C = halfedgeIter_CB->vertex();

    VertexIter vertexIter_M = newVertex();

    EdgeIter edgeIter_CM = newEdge();
    EdgeIter edgeIter_MB = newEdge();

    HalfedgeIter halfedgeIter_MC = newHalfedge();
    HalfedgeIter halfedgeIter_CM = newHalfedge();
    HalfedgeIter halfedgeIter_MB = newHalfedge();
    HalfedgeIter halfedgeIter_BM = newHalfedge();

    vertexIter_M->position = (vertexIter_B->position + vertexIter_C->position)/2.0;
    vertexIter_M->halfedge() = halfedgeIter_MB;

    edgeIter_CM->halfedge() = halfedgeIter_CM;
    edgeIter_MB->halfedge() = halfedgeIter_MB;

    halfedgeIter_MC->twin() = halfedgeIter_CM;
    halfedgeIter_CM->twin() = halfedgeIter_MC;
    halfedgeIter_MB->twin() = halfedgeIter_BM;
    halfedgeIter_BM->twin() = halfedgeIter_MB;

    halfedgeIter_MC->vertex() = vertexIter_M;
    halfedgeIter_CM->vertex() = vertexIter_C;
    halfedgeIter_MB->vertex() = vertexIter_M;
    halfedgeIter_BM->vertex() = vertexIter_B;

    halfedgeIter_MC->edge() = edgeIter_CM;
    halfedgeIter_CM->edge() = edgeIter_CM;
    halfedgeIter_MB->edge() = edgeIter_MB;
    halfedgeIter_BM->edge() = edgeIter_MB;

    vertexIter_B->halfedge() = halfedgeIter_BM;
    vertexIter_C->halfedge() = halfedgeIter_CM;

    FaceIter faceIter_1 = halfedgeIter_BC->face();
    FaceIter faceIter_2 = halfedgeIter_CB->face();

    if(!halfedgeIter_BC->isBoundary()) {
      HalfedgeIter halfedgeIter_CA = halfedgeIter_BC->next();
      HalfedgeIter halfedgeIter_AB = halfedgeIter_CA->next();

      VertexIter vertexIter_A = halfedgeIter_AB->vertex();


      FaceIter faceIter_A = newFace();
      FaceIter faceIter_B = newFace();

      faceIter_A->halfedge() = halfedgeIter_CA;
      faceIter_B->halfedge() = halfedgeIter_AB;

      EdgeIter edgeIter_AM = newEdge();

      HalfedgeIter halfedgeIter_MA = newHalfedge();
      HalfedgeIter halfedgeIter_AM = newHalfedge();

      edgeIter_AM->halfedge() = halfedgeIter_AM;

      halfedgeIter_AM->twin() = halfedgeIter_MA;
      halfedgeIter_MA->twin() = halfedgeIter_AM;
      halfedgeIter_AM->vertex() = vertexIter_A;
      halfedgeIter_MA->vertex() = vertexIter_M;
      halfedgeIter_AM->edge() = edgeIter_AM;
      halfedgeIter_MA->edge() = edgeIter_AM;

      halfedgeIter_AM->face() = faceIter_A;
      halfedgeIter_MC->face() = faceIter_A;
      halfedgeIter_CA->face() = faceIter_A;

      halfedgeIter_MA->face() = faceIter_B;
      halfedgeIter_AB->face() = faceIter_B;
      halfedgeIter_BM->face() = faceIter_B;

      halfedgeIter_AM->next() = halfedgeIter_MC;
      halfedgeIter_MC->next() = halfedgeIter_CA;
      halfedgeIter_CA->next() = halfedgeIter_AM;

      halfedgeIter_MA->next() = halfedgeIter_AB;
      halfedgeIter_AB->next() = halfedgeIter_BM;
      halfedgeIter_BM->next() = halfedgeIter_MA;

      deleteFace(faceIter_1);
    } else {
      halfedgeIter_BM->next() = halfedgeIter_MC;
      halfedgeIter_BM->face() = faceIter_1;
      halfedgeIter_MC->next() = halfedgeIter_BC->next();
      halfedgeIter_MC->face() = faceIter_1;
    }

    if(!halfedgeIter_CB->isBoundary()) {
      HalfedgeIter halfedgeIter_BD = halfedgeIter_CB->next();
      HalfedgeIter halfedgeIter_DC = halfedgeIter_BD->next();

      VertexIter vertexIter_D = halfedgeIter_DC->vertex();

      FaceIter faceIter_C = newFace();
      FaceIter faceIter_D = newFace();

      faceIter_C->halfedge() = halfedgeIter_DC;
      faceIter_D->halfedge() = halfedgeIter_BD;

      EdgeIter edgeIter_MD = newEdge();

      HalfedgeIter halfedgeIter_MD = newHalfedge();
      HalfedgeIter halfedgeIter_DM = newHalfedge();

      edgeIter_MD->halfedge() = halfedgeIter_MD;

      halfedgeIter_MD->twin() = halfedgeIter_DM;
      halfedgeIter_DM->twin() = halfedgeIter_MD;
      halfedgeIter_MD->vertex() = vertexIter_M;
      halfedgeIter_DM->vertex() = vertexIter_D;
      halfedgeIter_MD->edge() = edgeIter_MD;
      halfedgeIter_DM->edge() = edgeIter_MD;

      halfedgeIter_CM->face() = faceIter_C;
      halfedgeIter_MD->face() = faceIter_C;
      halfedgeIter_DC->face() = faceIter_C;

      halfedgeIter_DM->face() = faceIter_D;
      halfedgeIter_MB->face() = faceIter_D;
      halfedgeIter_BD->face() = faceIter_D;

      halfedgeIter_CM->next() = halfedgeIter_MD;
      halfedgeIter_MD->next() = halfedgeIter_DC;
      halfedgeIter_DC->next() = halfedgeIter_CM;

      halfedgeIter_DM->next() = halfedgeIter_MB;
      halfedgeIter_MB->next() = halfedgeIter_BD;
      halfedgeIter_BD->next() = halfedgeIter_DM;

      deleteFace(faceIter_2);

    } else {
      halfedgeIter_CM->next() = halfedgeIter_MB;
      halfedgeIter_CM->face() = faceIter_2;
      halfedgeIter_MB->next() = halfedgeIter_CB->next();
      halfedgeIter_MB->face() = faceIter_2;
    }

    deleteEdge(e0);
    deleteHalfedge(halfedgeIter_BC);
    deleteHalfedge(halfedgeIter_CB);

    return vertexIter_M;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
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

  }
}
