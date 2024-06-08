#include <string.h>
#include <sstream>

namespace csv_utils {

    bool to_bool (std::string s) {
        if (s == " True") {
            return true;
        }
        else {
            return false;
        }
    }
    
    double pymod(double n, double M) {
        return fmodl(((fmodl(n, M)) + M), M);
    }

    std::vector<double> keepOddRows(std::vector<double> matrix) {
        std::vector<double> result;
        for (size_t i = 0; i < matrix.size(); i += 2) {
            result.push_back(matrix[i]);
        }
        return result;
    }

    std::vector<bool> keepOddRows(std::vector<bool> matrix) {
        std::vector<bool> result;
        for (size_t i = 0; i < matrix.size(); i += 2) {
            result.push_back(matrix[i]);
        }
        return result;
    }
}