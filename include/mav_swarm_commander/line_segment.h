#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class LineSegment
{
    private:
        Eigen::Vector3d start_;
        Eigen::Vector3d end_;
        Eigen::ParametrizedLine<double, 3> line_; // for more info revise https://sciinstitute.github.io/shapeworks.pages/doxygen/class_eigen_1_1_parametrized_line.html
    public:
        /**
         * The constructor takes to to point and decide to which segement a point is near to
         * @param start and end
        */
        LineSegment(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

        /*
        * The memeber takes a aribitrary point and return how close is to the stat
        * or the end point
        * @return the distance regarding the values given in contructor
        */
        double pointDistance(const Eigen::Vector3d& p) const;

        /**
         * Give the relative position regarding the a waypoint given as start in the constructor
         * @note target here to see how far I am getting on
        */
        double relativePosition(const Eigen::Vector3d& p) const;

        /**
         * The member checks for me the point is insdie the segment I pass in
         * @return yes or no
        */
        bool isPointInside(const Eigen::Vector3d& p) const;

        /**
         * 
        */
        double absOffset(const Eigen::Vector3d& p) const;
};
