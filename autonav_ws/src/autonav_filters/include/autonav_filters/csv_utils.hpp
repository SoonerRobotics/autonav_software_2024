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
}