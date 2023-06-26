#pragma once

#include "CDT.h"
#include "VerifyTopology.h"
#include <iostream>

typedef float CoordType;
typedef CDT::Triangulation<CoordType> Triangulation;
typedef CDT::V2d<CoordType> V2d;
typedef CDT::Vertex<CoordType> Vertex;
typedef CDT::Triangle Triangle;
typedef CDT::Box2d<CoordType> Box2d;
typedef CDT::Index Index;
typedef CDT::Edge Edge;

inline Triangulation updateCDT(Triangulation m_cdt, std::vector<V2d> m_points, std::vector<Edge> m_edges)
{
#ifndef CDT_DONT_USE_BOOST_RTREE
    m_cdt = Triangulation(CDT::FindingClosestPoint::BoostRTree);
#else
    m_cdt = Triangulation(CDT::FindingClosestPoint::ClosestRandom, 10);
#endif
    int m_ptLimit = 9999;
    if (!m_points.empty())
    {
        std::vector<V2d> pts = m_points;
        //m_ptLimit < m_points.size()
        //? std::vector<V2d>(&m_points[0], &m_points[m_ptLimit])
        //: m_points;
        const std::vector<std::size_t> mapping = CDT::RemoveDuplicates(pts);
        m_cdt.insertVertices(pts);
        if (m_ptLimit >= m_points.size() && !m_edges.empty())
        {
            std::vector<Edge> edges = m_edges;
            /* m_edgeLimit < m_edges.size()
             ? std::vector<Edge>(&m_edges[0], &m_edges[m_edgeLimit])
             : m_edges;*/
            CDT::RemapEdges(edges, mapping);
            m_cdt.insertEdges(edges);
        }
        /*if (m_isRemoveOuterAndHoles)
            m_cdt.eraseOuterTrianglesAndHoles();
        else if (m_isRemoveOuter)
            m_cdt.eraseOuterTriangles();
        else if (m_isHideSuperTri)
            m_cdt.eraseSuperTriangle();*/
    }
    if (!CDT::verifyTopology(m_cdt))
    {
        /* QMessageBox errBox;
         errBox.setText(QStringLiteral("Triangulation has wrong topology"));
         errBox.exec();*/
        std::cout << "Triangulation has wrong topology";
    }
    return(m_cdt);
}