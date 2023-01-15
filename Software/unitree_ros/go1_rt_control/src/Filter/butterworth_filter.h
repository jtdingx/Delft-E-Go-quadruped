/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Build Date: 01/08/2018
 * Modify Date:
 *************************************************************/

#ifndef BUTTERWORTH_FILTER_H
#define BUTTERWORTH_FILTER_H

#include <iostream>

/**
 * @namespace gait
 */
    /**
     * @class ButterworthFiter
     * @brief force filter
     */

    class ButterworthFilter{
    public:
        /**
         * @brief constructor
         */
        ButterworthFilter();

        /**
         * @brief destructor
         */
        ~ButterworthFilter();

        /**
         * @brief Butterworth filter, second order model
         * @inputs force signal
         * @outputs force after filtering
         */
        double ForceFilter(const double input);

    private:
        int force_filter_count;
        double force_filter_raw_data[2], force_filter_filtering_data[3];
    };

#endif //BUTTERWORTH_FILTER_H
