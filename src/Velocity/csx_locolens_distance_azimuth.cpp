/**
* @file     csx_locolens_distance_azimuth.cpp
* @author   Nikita Anil
* @brief    This file defines functions to compute the azimuth angles and distances
*           between two specified points. It includes functions to convert angles
*           between radians and degrees, azimuth calculation using Vincenty's formula,
*           and distance determination between two points using Vincenty's formula,
*           which computes the geodesic distance providing highly accurate results
*           taking into consideration the ellipsoidal shape of the Earth, which accounts
*           for the flattening at the poles.The algorithm iteratively solves for the 
*           distance using trigonometric functions and convergence criteria to ensure 
*           precision.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise self-localization 
*                               using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software modules 
*                               and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

// system header file
#include <cmath>
#include <utility>

// local header file
#include "Velocity/csx_locolens_distance_azimuth.h"


/**
* @brief - This function takes an angle in radians as input and returns the equivalent
*          angle in degrees by multiplying with a factor (180 / PI).
* @param   rad: The angle in radians that is to be converted.
* @return  The angle in degrees.
*/
constexpr double DistanceAzimuth::Rad2Deg(double rad)
{
    return rad * (180 / kPi);
}

/**
* @brief - This function takes an angle in degrees as input and returns the equivalent
*          angle in radians by multiplying with a factor (PI / 180).
* @param   deg: The angle in degrees that is to be converted.
* @return  The angle in degrees.
*/
constexpr double DistanceAzimuth::Deg2Rad(double deg)
{
    return deg * (kPi / 180);
}

/**
* @brief - This function uses Vincenty's formula to determine the azimuth angle (the
*          bearing) from a starting point to an endpoint defined by latitude and 
*          longitude coordinates. It considers the Earth's flattening effect for 
*          precise azimuth computation between the two points. The result is returned
*          in degrees, adjusted to be in the range [0, 360) degrees.
* @param  point1: A pair of double values of the latitude and longitude of the
*                 starting point in degrees.
* @param  point2: A pair of double values of the latitude and longitude of the
*                 destination point in degrees.
* @return  The azimuth angle in degrees between the two points, ranging from 0
*          to 360 degrees.
*/
double DistanceAzimuth::CalculateAzimuth(const std::pair<double, double> &point1,
                        const std::pair<double, double> &point2)
{
    double latitude_1 = point1.first;
    double longitude_1 = point1.second;
    double latitude_2 = point2.first;
    double longitude_2 = point2.second;

    // Convert latitude and longitude from degrees to radians
    double latitude_1_rad = Deg2Rad(latitude_1);
    double latitude_2_rad = Deg2Rad(latitude_2);
    double delta_longitude_radian = Deg2Rad(longitude_2 - longitude_1);

    // Reduced latitude
    double reduced_latitude_1 = atan((1 - kFlattening) * tan(latitude_1_rad));
    double reduced_latitude_2 = atan((1 - kFlattening) * tan(latitude_2_rad));

    // Azimuth calculation using Vincenty's formula
    double azimuth = atan2((cos(reduced_latitude_2) *
                            sin(delta_longitude_radian)),
                           (cos(reduced_latitude_1) * sin(reduced_latitude_2) -
                            sin(reduced_latitude_1) * cos(reduced_latitude_2) *
                                cos(delta_longitude_radian)));

    // Convert azimuth from radians to degrees
    double azimuth_degree = Rad2Deg(azimuth);

    if(azimuth_degree < 0)
    {
        azimuth_degree += 360;
    }

    return azimuth_degree;
}

/**
* @brief - This function takes two points in latitude and longitude coordinates and 
*          calculates the shortest distance between them over the Earth's surface. It
*          uses iterative calculations with Vincenty's formulae, which are highly
*          accurate for distances on an ellipsoid.
* @param  point1: A pair of double values of the latitude and longitude of the
*                 starting point in degrees.
* @param  point2: A pair of double values of the latitude and longitude of the
*                 destination point in degrees.
* @return  The distance between the two points in meters.
*/
double DistanceAzimuth::Vincenty(const std::pair<double, double> &point1,
                                 const std::pair<double, double> &point2)
{
    // Extract latitude and longitude from the input points
    double latitude_1 = point1.first;
    double longitude_1 = point1.second;
    double latitude_2 = point2.first;
    double longitude_2 = point2.second;

    // Convert latitude to 'reduced latitude' based on Earth's flattening
    double reduced_latitude_1 = atan((1 - kFlattening) *
                                     tan(Deg2Rad(latitude_1)));
    double reduced_latitude_2 = atan((1 - kFlattening) *
                                     tan(Deg2Rad(latitude_2)));
    double delta_longitude = Deg2Rad(longitude_2 - longitude_1);
    double lambda_var = delta_longitude;
    const double TOL = 1e-12;   // Convergence tolerance
    const int ITER_LIMIT = 100; // Maximum number of iterations

    double sin_lambda, cos_lambda, sin_sigma, cos_sigma, sigma;
    double sin_alpha, cos2_alpha, cos2_sigma_m;
    double c_value, lambda_prev;

    // Iterate to solve for lambda using Vincenty's iterative method
    for (int i = 0; i < ITER_LIMIT; ++i)
    {

        // Recompute values for current lambda
        sin_lambda = sin(lambda_var);
        cos_lambda = cos(lambda_var);

        // Calculate sigma, the angular separation between the two points
        sin_sigma = sqrt(
            pow(cos(reduced_latitude_2) * sin_lambda, 2) +
            pow(cos(reduced_latitude_1) * sin(reduced_latitude_2) -
                    sin(reduced_latitude_1) * cos(reduced_latitude_2) *
                        cos_lambda,
                2));

        // Calculate the angle between the points
        cos_sigma = sin(reduced_latitude_1) * sin(reduced_latitude_2) +
                    cos(reduced_latitude_1) * cos(reduced_latitude_2) *
                        cos_lambda;
        sigma = atan2(sin_sigma, cos_sigma);

        // Calculate sin and cos of alpha, the azimuth angle
        sin_alpha = cos(reduced_latitude_1) *
                    cos(reduced_latitude_2) * sin_lambda / sin_sigma;
        cos2_alpha = 1 - pow(sin_alpha, 2);

        // Compute cos(2 * sigma_m), the latitude at the midpoint
        cos2_sigma_m = cos_sigma - 2 * sin(reduced_latitude_1) *
                                       sin(reduced_latitude_2) / cos2_alpha;

        // Calculate C value based on Earth's flattening
        c_value = kFlattening / 16 * cos2_alpha *
                  (4 + kFlattening * (4 - 3 * cos2_alpha));

        // Update lambda for next iteration
        lambda_prev = lambda_var;
        lambda_var = delta_longitude +
            (1 - c_value) * kFlattening * sin_alpha * 
                (sigma + c_value * sin_sigma * 
                    (cos2_sigma_m + c_value * cos_sigma *
                        (-1 + 2 * pow(cos2_sigma_m, 2))));
        if (fabs(lambda_var - lambda_prev) < TOL)
        {
            break;
        }
    }

    // Calculate Earth's radius of curvature in the direction of the geodesic
    double ellipsoid_curvature = cos2_alpha * (pow(kSemiMajorAxis, 2) - 
                                    pow(kSemiMinorAxis, 2)) /
                                        pow(kSemiMinorAxis, 2);

    // Calculate the factor to apply to Earth's 
    // semi-minor axis to get the distance
    double radius_reduction_factor = 1 + ellipsoid_curvature / 16384 *
        (4096 + ellipsoid_curvature * (-768 + ellipsoid_curvature *
            (320 - 175 * ellipsoid_curvature)));

    // Calculate the second term factor in the distance formula
    double second_term_factor = ellipsoid_curvature / 1024 *
        (256 + ellipsoid_curvature * (-128 + ellipsoid_curvature *
                                        (74 - 47 * ellipsoid_curvature)));

    // Compute the delta sigma term, correcting for Earth's curvature
    double delta_sigma = second_term_factor * sin_sigma * (cos2_sigma_m + 
        second_term_factor / 4 * (cos_sigma * (-1 + 2 * pow(cos2_sigma_m, 2)) -
            second_term_factor / 6 * cos2_sigma_m *
                (-3 + 4 * pow(sin_sigma, 2)) * 
                    (-3 + 4 * pow(cos2_sigma_m, 2))));

    // Final distance in meters, taking Earth's semi-minor axis as the base
    double distance_in_meters = kSemiMinorAxis *
                                radius_reduction_factor *
                                (sigma - delta_sigma);

    return distance_in_meters; // Return the calculated distance
}
