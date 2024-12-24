#include "mex.h"
#include <cmath>

// 计算矩阵的二范数
double calculateNorm(const mxArray *matrix) {
    // 获取矩阵数据指针
    double *data = mxGetPr(matrix);
    
    // 获取矩阵的行数和列数
    mwSize rows = mxGetM(matrix);
    mwSize cols = mxGetN(matrix);
    
    // 计算二范数
    double norm = 0.0;
    for (mwSize i = 0; i < rows * cols; ++i) {
        norm += data[i] * data[i];
    }
    return std::sqrt(norm);
}

// mexFunction 是 MEX 文件的入口函数
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // 检查输入参数的数量是否正确
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:matrix_norm:invalidNumInputs", "One input required.");
    }
    
    // 检查输出参数的数量是否正确
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("MATLAB:matrix_norm:invalidNumOutputs", "Too many output arguments.");
    }
    
    // 检查输入是否为非空的二维矩阵
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetNumberOfDimensions(prhs[0]) != 2) {
        mexErrMsgIdAndTxt("MATLAB:matrix_norm:inputNotRealMatrix", "Input must be a non-complex double matrix.");
    }
    
    // 计算矩阵的二范数
    double norm = calculateNorm(prhs[0]);
    
    // 创建输出结果
    plhs[0] = mxCreateDoubleScalar(norm);
}
