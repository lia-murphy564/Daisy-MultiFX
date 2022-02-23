#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include "json.hpp"
#include <iostream>
#include <iomanip>

using json = nlohmann::json;

json data_ble = {
    {"Sender", 0},
    {"1", {
        {"label", "Pot1"},
        {"type", "Pot"},
        {"value", 40}
    }},
    {"2", {
        {"label", "Pot2"},
        {"type", "Pot"},
        {"value", 79}
    }},
    {"3", {
        {"label", "Po3"},
        {"type", "Pot"},
        {"value", 37}
    }},    
    {"4", {
        {"label", "Switch1"},
        {"type", "Switch"},
        {"value", 0}
    }},
    {"5", {
        {"label", "Switch2"},
        {"type", "Switch"},
        {"value", 0}
    }},
    {"6", {
        {"label", "Footswitch"},
        {"type", "Switch"},
        {"value", 0}
    }}
};

json data_gpio = {
    {"Sender", 1},
    {"1", {
        {"label", "Pot1"},
        {"type", "Pot"},
        {"value", 0}
    }},
    {"2", {
        {"label", "Pot2"},
        {"type", "Pot"},
        {"value", 0}
    }},
    {"3", {
        {"label", "Po3"},
        {"type", "Pot"},
        {"value", 0}
    }},    
    {"4", {
        {"label", "Switch1"},
        {"type", "Switch"},
        {"value", 0}
    }},
    {"5", {
        {"label", "Switch2"},
        {"type", "Switch"},
        {"value", 0}
    }},
    {"6", {
        {"label", "Footswitch"},
        {"type", "Switch"},
        {"value", 0}
    }}
};

struct param {
	std::string label;
	std::string type;
	uint16_t val;
    int index;

    void print() {
        std::cout << index << " " << label << " " << type << " " << val << "\n";
    }

    bool operator<(const param& p) const
    {
        if (index < p.index)
            return true;
        else return false;
    }
    bool operator==(const param& p) const
    {
        if (label == p.label && type == p.type && val == p.val)
            return true;
        else return false;
    }
};

class ParameterTree {

private:
    std::priority_queue<param> pq;
    int num_params;

public:
    ParameterTree() {
        num_params = 10;
    };
    ~ParameterTree() {};

    void push(param p) {
        pq.push(p);
    }
    void print_tree() {
        while (!pq.empty()) {
            std::cout << pq.top().index << " " << pq.top().label << " " << pq.top().type << " " << pq.top().val << "\n";
            pq.pop();
        }
    }
    bool is_empty() {
        return (pq.empty() ? true : false);
    }
    int get_num_params() { return num_params; }
    void set_num_params(int n) { num_params = n; }

};

class ParameterController {
    private:
        ParameterTree pt;
        
        json data_gpio;
        json data_ble;
        json data;
        
        bool is_sending_gpio;
        bool is_sending_ble;

        int num_params;

        param curr;


    protected:

        void update_param_at_index(int i) {
            curr.index = i;
            std::string idx = std::to_string(curr.index);
            curr.label = data.at(idx).value("label","");
            curr.type = data.at(idx).value("type","");
            curr.val = data.at(idx).value("value",0);
            pt.push(curr);
        }

    public:

        ParameterController() {};
        ~ParameterController() {};

        void set_data_gpio(json j) { data_gpio = j; }
        void set_data_ble(json j) { data_ble = j; }

        //void set_num_params(int n) { num_params = n; }

        void update_parameters() {

            num_params = data_gpio.size() - 1;
            pt.set_num_params(num_params);

            // check ble sending state
            is_sending_ble = data_ble.value("Sender",0);
            is_sending_gpio = data_gpio.value("Sender",0);

            // check sending state concurrency
            if (is_sending_ble) {
                data = data_ble;
            }
            else if (is_sending_gpio) {
                data = data_gpio;
            }
            else { // sending states are equal
                data = data_gpio;
            }

            //update parameter tree
            for (int i = 0; i < num_params; i++) {
                update_param_at_index(i + 1);
            }

            pt.print_tree();
        }
};

// int dsy_gpio_read() {
//     return 63;
// }

// int main(void) {

//     // read gpio value into json
//     for (int i = 0; i < data_gpio.size()-1; i++) {
//         std::string idx = std::to_string(i + 1);
//         data_gpio[idx]["value"] = i;//dsy_gpio_read();
//     }

//     ParameterController parameterController;

//     parameterController.set_data_ble(data_ble);
//     parameterController.set_data_gpio(data_gpio);
    

//     //case int index;


    
//     while(true) {
//         parameterController.update_parameters();
//     }
    
//     return 1;
// }