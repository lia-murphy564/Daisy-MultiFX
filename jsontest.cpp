#include <iostream>
#include "json.hpp"

using json = nlohmann::json;

struct param_list {
    param * curr;
    param * next;
    
}

struct param {
	std::string label;
	std::string type;
	uint16_t val;

    param();

    param(std::string _label, std::string _type, uint16_t _val) {
        label = _label;
        type = _type;
        val = _val;
    }
};

int main() {

	json data = 
	{
		{"label", "pot1"},
		{"type", "pot"},
		{"val", 50}
	};
	
	param p;
	p.label = data.value("label", "");
	p.type = data.value("type", "");
	p.val = data.value("val", 0);

    for () {
        
    }

	std::cout << p.label << " " << p.type << " " << p.val << "\n";
    return 0;
}