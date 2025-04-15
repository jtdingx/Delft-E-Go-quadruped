/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Build Date: 01/08/2018
 * Modify Date:
 *************************************************************/

#include "butterworth_filter.h"

/**
 * @namespace gait
 */
    /**
     * @class ButterworthFilter
     */

    ButterworthFilter::ButterworthFilter() {
        force_filter_count = 0;
        force_filter_filtering_data[0] = 0.0;
        force_filter_filtering_data[1] = 0.0;
        force_filter_filtering_data[2] = 0.0;
        force_filter_raw_data[0] = 0.0;
        force_filter_raw_data[1] = 0.0;
    }

    ButterworthFilter::~ButterworthFilter() {}

    /**
     * @brief filter
     * @param input
     * @param output
     * @return
     */
    double ButterworthFilter::ForceFilter(const double input) {
        const double a[3] = {1.0, -1.6498, 0.7022};
        const double b[2] = {0.0, 0.0521};


        if(0 == force_filter_count){

            force_filter_filtering_data[2] = input;
            force_filter_raw_data[1] = input;

            force_filter_count++;
        } else if(1 == force_filter_count){

            force_filter_filtering_data[1] = force_filter_filtering_data[2];
            force_filter_filtering_data[2] = input;

            force_filter_raw_data[0] = force_filter_raw_data[1];
            force_filter_raw_data[1] = input;

            force_filter_count++;
        }else{

            force_filter_raw_data[0] = force_filter_raw_data[1];
            force_filter_raw_data[1] = input;

            force_filter_filtering_data[0] = force_filter_filtering_data[1];
            force_filter_filtering_data[1] = force_filter_filtering_data[2];
            force_filter_filtering_data[2] = b[0]*force_filter_raw_data[1] + b[1]*force_filter_raw_data[0]
                    - a[1]*force_filter_filtering_data[1] -a[2]*force_filter_filtering_data[0];
        }

//        std::cout << "raw data: " << output << std::endl;


        return force_filter_filtering_data[2];
    }

