#include "mex.h"
#include <cstring>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // 检查输入输出数量
    if (nrhs != 2) {  // 需要两个输入：SimScenario 对象和字符串
        mexErrMsgIdAndTxt("MATLAB:mexFunction:invalidNumInputs", "Two inputs required.");
    }
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:invalidNumOutputs", "Too many output arguments.");
    }

    // 检查第一个输入是否为对象
    if (!mxIsClass(prhs[0], "SimScenario")) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:invalidInputType", "First input must be a SimScenario object.");
    }

    // 检查第二个输入是否为字符串
    if (!mxIsChar(prhs[1])) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:inputNotString", "Second input must be a string.");
    }

    // 获取字符串内容
    char *inputString = mxArrayToString(prhs[1]);
    if (inputString == nullptr) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:conversionFailed", "Could not convert input to string.");
    }

    // 获取 SimScenario 对象的 net 属性
    mxArray *netObj = mxGetProperty(prhs[0], 0, "net");
    if (netObj == nullptr) {
        mxFree(inputString);  // 释放字符串内存
        mexErrMsgIdAndTxt("MATLAB:mexFunction:propertyNotFound", "SimScenario object does not have a net property.");
    }

    // 创建参数数组，传递 net 对象和字符串
    mxArray *lhs[1];  // 用于接收返回值
    mxArray *rhs[2];  // 用于传递 net 对象和字符串
    rhs[0] = netObj;  // net 对象
    rhs[1] = mxCreateString(inputString);  // 可变大小的字符串

    // 调用 net 对象的 isInNet 方法
    mexCallMATLAB(1, lhs, 2, rhs, "isInNet");

    // 返回结果
    plhs[0] = lhs[0];

    // 释放字符串内存
    mxFree(inputString);
}
