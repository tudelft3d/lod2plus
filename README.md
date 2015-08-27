# lod2plus
Automatic generation of simple interiors for CityGML LOD2

Currently CityGML only features interiors for the highest Level-of-Detail (LoD4). In his master thesis project for Geomatics (see http://www.geomatics.tudelft.nl/), Roeland Boeters extended CityGML to feature interiors for LoD1-LoD4. This is necessary because there are numerous applications that need interiors but not with the amount of detail in LoD4.

This project features the source code that is developed to enhance CityGML LoD2 to LoD2+, by modelling solid volumes for each storey of a building. Furthermore as an application it is used for the determination of the net internal area of residential units which is an important quantity in the Netherlands through the key register BAG and WOZ.

You can read more (and cite) the publication:

> Automatically enhancing CityGML LOD2 models with a corresponding indoor geometry. Roeland Boeters, Ken Arroyo Ohori, Filip Biljecki and Sisi Zlatanova. *International Journal of Geographical Information Science*, 2015. In press. [ [PDF] ](https://3d.bk.tudelft.nl/ken/files/15_ijgis_roeland.pdf) [ [DOI] ](http://dx.doi.org/10.1080/13658816.2015.1072201)

The source code is built upon CGAL and therefore the [GPL](http://www.gnu.org/copyleft/gpl.html) applies to this project.
