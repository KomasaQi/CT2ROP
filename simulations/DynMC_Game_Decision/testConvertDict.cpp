#include "mex.h"
#include <unordered_map>
#include <vector>
#include <string>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // 检查输入参数数量
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:invalidNumInputs", "One input required.");
    }


    // 获取 containers.Map 的 keys 和 values
    mxArray *keysArray = mxGetProperty(prhs[0], 0, "keys");
    mxArray *valuesArray = mxGetProperty(prhs[0], 0, "values");

    if (!keysArray || !valuesArray) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:propertyNotFound", "Failed to retrieve keys or values from containers.Map.");
    }

    // 将 keysArray 和 valuesArray 转换为单元数组
    keysArray = mxGetCell(keysArray, 0);
    valuesArray = mxGetCell(valuesArray, 0);

    // 获取键和值的数量
    mwSize numKeys = mxGetNumberOfElements(keysArray);

    // 创建 C++ unordered_map
    std::unordered_map<std::string, std::vector<mxArray*>> cppMap;

    // 遍历 keys 和 values，将其存储到 C++ unordered_map 中
    for (mwSize i = 0; i < numKeys; ++i) {
        // 获取键并转换为 C++ 字符串
        mxArray *keyElement = mxGetCell(keysArray, i);
        char *keyStr = mxArrayToString(keyElement);

        // 获取对应的值 (cell 数组)
        mxArray *valueElement = mxGetCell(valuesArray, i);

        // 检查值是否为 cell 数组
        if (!mxIsCell(valueElement)) {
            mxFree(keyStr);
            mexErrMsgIdAndTxt("MATLAB:mexFunction:valueNotCell", "Values must be cell arrays.");
        }

        // 遍历 cell 数组中的对象并存储到 vector 中
        mwSize numObjects = mxGetNumberOfElements(valueElement);
        std::vector<mxArray*> objectVector;

        for (mwSize j = 0; j < numObjects; ++j) {
            mxArray *object = mxGetCell(valueElement, j);
            objectVector.push_back(object);
        }

        // 将键值对插入到 C++ unordered_map 中
        cppMap[keyStr] = objectVector;

        // 释放 keyStr 内存
        mxFree(keyStr);
    }

    // 处理后的 C++ unordered_map 可以用于进一步操作
    // 举例：将哈希表中的键打印出来
    for (const auto &pair : cppMap) {
        mexPrintf("Key: %s\n", pair.first.c_str());
        mexPrintf("Number of objects in cell: %zu\n", pair.second.size());
        for (auto obj : pair.second) {
            // 这里你可以处理 obj（MATLAB 对象）进行其他操作
        }
    }

    // 如果需要返回数据给 MATLAB，使用 plhs
}
