Currently CityGML only features interiors for the highest Level-of-Detail (LoD4). In my master thesis project for Geomatics (see http://www.geomatics.tudelft.nl/) I extended CityGML to feature interiors for LoD1-LoD4. This is necessary because there are numerous applications that need interiors but not with the amount of detail as in LoD4.

This project features the source code that is developed to enhance CityGML LoD2 to LoD2+, by modelling solid volumes for each storey of a premises. Furthermore as an application it is used for the determination of the net internal area of residential units which is an important quantity in the Netherlands through the key register BAG and WOZ.

The source code is built upon CGAL and therefore GPL applies to this project.