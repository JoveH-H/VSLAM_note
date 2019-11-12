#include <iostream>
#include <fstream>
#include "ceres/ceres.h"

#include "SnavelyReprojectionError.h"
#include "common/BALProblem.h"
#include "common/BundleParams.h"


using namespace ceres;

void SetLinearSolver(ceres::Solver::Options* options, const BundleParams& params)
{
    CHECK(ceres::StringToLinearSolverType(params.linear_solver, &options->linear_solver_type));
    CHECK(ceres::StringToSparseLinearAlgebraLibraryType(params.sparse_linear_algebra_library, &options->sparse_linear_algebra_library_type));
    CHECK(ceres::StringToDenseLinearAlgebraLibraryType(params.dense_linear_algebra_library, &options->dense_linear_algebra_library_type));
}


void SetOrdering(BALProblem* bal_problem, ceres::Solver::Options* options, const BundleParams& params)
{
    const int num_points = bal_problem->num_points();
    const int point_block_size = bal_problem->point_block_size();
    double* points = bal_problem->mutable_points();

    const int num_cameras = bal_problem->num_cameras();
    const int camera_block_size = bal_problem->camera_block_size();
    double* cameras = bal_problem->mutable_cameras();


    if (params.ordering == "automatic")
        return;

    ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;

    // The points come before the cameras
    for(int i = 0; i < num_points; ++i)
       ordering->AddElementToGroup(points + point_block_size * i, 0);
       
    
    for(int i = 0; i < num_cameras; ++i)
        ordering->AddElementToGroup(cameras + camera_block_size * i, 1);

    options->linear_solver_ordering.reset(ordering);

}

void SetMinimizerOptions(Solver::Options* options, const BundleParams& params){
    options->max_num_iterations = params.num_iterations;
    options->minimizer_progress_to_stdout = true;
    options->num_threads = params.num_threads;
    // options->eta = params.eta;
    // options->max_solver_time_in_seconds = params.max_solver_time;
    
    CHECK(StringToTrustRegionStrategyType(params.trust_region_strategy,
                                        &options->trust_region_strategy_type));
    
}

void SetSolverOptionsFromFlags(BALProblem* bal_problem,
                               const BundleParams& params, Solver::Options* options){
    SetMinimizerOptions(options,params);
    SetLinearSolver(options,params);
    // 设置变量排序
    SetOrdering(bal_problem, options,params);
}

void BuildProblem(BALProblem* bal_problem, Problem* problem, const BundleParams& params)
{
    const int point_block_size = bal_problem->point_block_size();
    const int camera_block_size = bal_problem->camera_block_size();

    // 构建points、cameras参数类型的优化变量
    double* points = bal_problem->mutable_points(); 
    double* cameras = bal_problem->mutable_cameras();

    // Observations is 2 * num_observations long array observations
    // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x 
    // and y position of the observation. 
    const double* observations = bal_problem->observations();

    // 有观测数量有多少就有多少误差项
    for(int i = 0; i < bal_problem->num_observations(); ++i){
        CostFunction* cost_function;

        // Each Residual block takes a point and a camera as input 
        // and outputs a 2 dimensional Residual
        // 在这里构建代价函数costfunction每个误差块是以一个points和一个cameras为输入的
        cost_function = SnavelyReprojectionError::Create(observations[2*i + 0], observations[2*i + 1]);

        // 设置是否开启核函数
        LossFunction* loss_function = params.robustify ? new HuberLoss(1.0) : NULL;

        // Each observatoin corresponds to a pair of a camera and a point 
        // which are identified by camera_index()[i] and point_index()[i]
        // respectively.
        double* camera = cameras + camera_block_size * bal_problem->camera_index()[i];
        double* point = points + point_block_size * bal_problem->point_index()[i];

     
        problem->AddResidualBlock(cost_function, loss_function, camera, point);
    }

}

void SolveProblem(const char* filename, const BundleParams& params)
{
    // 读取文件
    BALProblem bal_problem(filename);

    // 显示信息
    std::cout << "bal problem file loaded..." << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observatoins. " << std::endl;

    // 储存初始的（未BA前的）3d点云位置以及相机位置
    if(!params.initial_ply.empty()){
        bal_problem.WriteToPLYFile(params.initial_ply);
    }

    std::cout << "beginning problem..." << std::endl;
    
    // 给camera,points都加上噪音
    srand(params.random_seed);
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma,
                        params.point_sigma);

    std::cout << "Normalization complete..." << std::endl;
    
    // 开始构建最小二乘问题
    Problem problem;
    BuildProblem(&bal_problem, &problem, params);

    std::cout << "the problem is successfully build.." << std::endl;
   
    // 配置求解器
    Solver::Options options;
    SetSolverOptionsFromFlags(&bal_problem, params, &options);
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // 导出未BA后的3d点云位置以及相机位置 
    if(!params.final_ply.empty()){
        bal_problem.WriteToPLYFile(params.final_ply); 
    }
}

int main(int argc, char** argv)
{    
    // 设置参数
    BundleParams params(argc,argv);
   
    google::InitGoogleLogging(argv[0]);
    std::cout << params.input << std::endl;
    if(params.input.empty()){
        std::cout << "Usage: bundle_adjuster -input <path for dataset>";
        return 1;
    }

    // 求解BA
    SolveProblem(params.input.c_str(), params);
 
    return 0;
}
