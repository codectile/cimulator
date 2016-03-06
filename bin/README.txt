This README is all about vehicle collision database.

Q:How to install vehicle collision database?

A:There is a file named as SAMPVehicleCOLs.ccf
  copy it
  goto your "server\scriptfiles\" directory and create a folder with the name "cimulator"
  Now, paste the database in the "cimulator" folder you just created
  that's it

Additional Information:
=======================

The database, SAMPVehicleCOLs.ccf has an extension .ccf which stands for
"(c)imulator (c)ollision (f)ormat".
Database Size: 1,84,581 bytes
Number of Vehicle Data: 211

Structure:
=======================


 TSphere:
 ========
 TVector {12}						- center of the sphere
 float {4}						- radius of the sphere



 TBox:
 =====
 TVector {12}						- center
 TVector {12}						- half extents



 TFace:
 ======
 TVector {12}						- vertex 0
 TVector {12}						- vertex 1
 TVector {12}						- vertex 2



 Header:
 =======
 * char {4}						- "CCF1" which stands for cimulator collision format (version 1)
 * uint8 {1}						- number of supported models, always 211



 Body:
 =====
 * uint16 {2}						- modelid
 * uint16 {2}						- number of spheres
 * uint16 {2}						- number of boxes
 * uint16 {2}						- number of faces
 * TSphere []{16 * number of spheres}			- sphere data block
 * TBox []{36 * number of boxes}			- boxes data block
 * TFace []{36 * number of faces}			- faces data block
