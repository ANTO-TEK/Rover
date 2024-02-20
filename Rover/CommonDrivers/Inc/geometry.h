/* 
 * Course: IoT and embedded system design 2023/2024
 * 
 * Lecturer: Francesco Moscato	fmoscato@unisa.it
 *
 * Group:
 * Adinolfi Teodoro    0622701902	   t.adinolfi2@studenti.unisa.it
 * Amato Emilio        0622701903      e.amato16@studenti.unisa.it             
 * Bove Antonio        0622701898      a.bove57@studenti.unisa.it 
 * Guarini Alessio     0622702042	   g.guarini7@studenti.unisa.it
 *
 * Copyright (C) 2024 - All Rights Reserved
 *
 * This file is part of CommonDrivers.
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of 
 * the GNU General Public License as published by the Free Software Foundation, either version 
 * 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ContestOMP. 
 * If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file geometry.h
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief Header file for geometric coordinate representations.
 *
 * This header file provides the definitions and structures for representing
 * points in Cartesian, Polar, and Spherical coordinate systems.
 */

#ifndef INC_GEOMETRY_H_
#define INC_GEOMETRY_H_

#include <stdint.h>

/**
 * @brief Type definition for Cartesian coordinates.
 *
 * Defines a type for Cartesian3D coordinates using a 32-bit float.
 */
typedef float cartesian3d_coordinate_t;

/**
 * @brief Type definition for Cartesian2D coordinates.
 *
 * Defines a type for Cartesian coordinates using a 16-bit unsigned integer.
 */
typedef int8_t cartesian2d_coordinate_t;


/**
 * @brief Type definition for radial coordinates in polar and spherical systems.
 *
 * Defines a type for radial coordinates using a double precision floating point.
 */
typedef double radial_coordinate_t;

/**
 * @brief Structure for 2D Cartesian coordinates.
 *
 * This structure represents a point in 2-dimensional Cartesian coordinate system.
 */
typedef struct {
	cartesian2d_coordinate_t x; /**< @brief The x-coordinate */
	cartesian2d_coordinate_t y; /**< @brief The y-coordinate */
} Cartesian2D;

/**
 * @brief Structure for 3D Cartesian coordinates.
 *
 * This structure represents a point in 3-dimensional Cartesian coordinate system.
 */
typedef struct {
	cartesian3d_coordinate_t x; /**< @brief The x-coordinate */
	cartesian3d_coordinate_t y; /**< @brief The y-coordinate */
	cartesian3d_coordinate_t z; /**< @brief The z-coordinate */
} Cartesian3D;

/**
 * @brief Structure for Polar coordinates.
 *
 * This structure represents a point in polar coordinate system.
 */
typedef struct {
    radial_coordinate_t radius; /**< @brief The radius from the origin */
    radial_coordinate_t theta;  /**< @brief The angle from the reference direction */
} Polar;

/**
 * @brief Structure for Spherical coordinates.
 *
 * This structure represents a point in spherical coordinate system.
 */
typedef struct {
    radial_coordinate_t radius; /**< @brief The radius from the origin */
    radial_coordinate_t theta;  /**< @brief The azimuthal angle */
    radial_coordinate_t phi;    /**< @brief The polar angle */
} Spherical;

#endif /* INC_GEOMETRY_H_ */
