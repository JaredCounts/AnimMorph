#include "loadMesh.h"

#include <iostream>

#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include "rapidxml_print.hpp"

#include "../linearAlgebra.h"
#include "outlineToMesh2D.h"

namespace LoadMesh
{
Mesh2D LoadMesh(const std::string &svgFilePath)
{
	//// 1. load the file into string
	//std::string svgContent = GetFileContents(svgFilePath.data());

	//// 2. parse string as xml
	//rapidxml::xml_document<> doc; // character type defaults to char
	//doc.parse<0>(svgContent.data()); // 0 means default parse flags


	// 1. parse file
	rapidxml::file<> xmlFile(svgFilePath.data()); // Default template is char
	rapidxml::xml_document<> doc;

	doc.parse<0>(xmlFile.data());

	// 2. extract polyline
	rapidxml::xml_node<> *node = doc.first_node();
	std::cout << node->name() << '\n';
	assert(strcmp(node->name(), "svg") == 0);

	rapidxml::xml_node<> *child = node->first_node();

	while (strcmp(child->name(), "polyline") != 0)
	{
		std::cout << child->name() << '\n';
		child = child->next_sibling();

		if (child == node->last_node())
		{
			break;
		}
	}

	assert(strcmp(child->name(), "polyline") == 0);

	std::string pointsText;

	for (rapidxml::xml_attribute<> *attr = child->first_attribute();
		attr; attr = attr->next_attribute())
	{
		if (strcmp(attr->name(), "points") == 0)
		{
			pointsText = attr->value();
			break;
		}
	}

	std::cout << pointsText << '\n';

	// separate by space
	std::istringstream spaceSep(pointsText);
	std::string token;

	Matrix2Xf points;

	while (std::getline(spaceSep, token, ' ')) {

		// separate by comma
		std::istringstream commaSep(token);

		std::string vertexA, vertexB;
		std::getline(commaSep, vertexA, ',');
		std::getline(commaSep, vertexB, ',');

		// std::cout << vertexA << " and " << vertexB << '\n';
		points.conservativeResize(points.rows(), points.cols() + 1);

		points.col(points.cols() - 1) = Vector2f(std::stof(vertexA), std::stof(vertexB));
	}

	// XXX: remove duplicate consecutive points, remove colinear points
	return OutlineToMesh2D::OutlineToMesh2D(points);
}

}