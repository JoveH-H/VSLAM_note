#include <Eigen/Core>
#include "ceres/autodiff.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"

#include "tools/rotation.h"
#include "common/projection.h"

class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {}

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update) override
    {
        Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
        _estimate += v;
    }

};


class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() {}

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update) override
    {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }

    virtual void computeError() override   // 覆盖基类函数，使用operator()计算代价函数
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> (vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> (vertex(1));

        (*this) (cam->estimate().data(), point->estimate().data(), _error.data());

    }

    // 为了使用 Ceres 求导功能而定义的函数，让本类成为拟函数类
    template<typename T>
    bool operator() (const T* camera, const T* point, T* residuals) const
    {
        T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - T(measurement() (0));
        residuals[1] = predictions[1] - T(measurement() (1));

        return true;
    }


    virtual void linearizeOplus() override
    {
        // 使用数字雅可比矩阵
        // BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>::linearizeOplus();
        // return;

        // using autodiff from ceres. Otherwise, the system will use g2o numerical diff for Jacobians

        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> (vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> (vertex(1));
        typedef ceres::internal::AutoDiff<EdgeObservationBAL, double, VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;

        Eigen::Matrix<double, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
        Eigen::Matrix<double, Dimension, VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;
        double *parameters[] = { const_cast<double*> (cam->estimate().data()), const_cast<double*> (point->estimate().data()) };
        double *jacobians[] = { dError_dCamera.data(), dError_dPoint.data() };
        double value[Dimension];
        // Ceres 中的自动求导函数用法，需要提供 operator() 函数成员
        bool diffState = BalAutoDiff::Differentiate(*this, parameters, Dimension, value, jacobians);

        // copy over the Jacobians (convert row-major -> column-major)
        if (diffState)
        {
            _jacobianOplusXi = dError_dCamera;
            _jacobianOplusXj = dError_dPoint;
        }
        else
        {
            assert(0 && "Error while differentiating");
            _jacobianOplusXi.setZero();
            _jacobianOplusXi.setZero();
        }
    }
};
