World Magnetic Model WMM2025
========================================================
Date December 17, 2024

WMM.COF					WMM2025 Coefficients file
						(Replace old WMM.COF (WMM2020 
						 or WMM2015) file with this)

1. Installation Instructions
==========================

WMM2025 GUI
-----------

Go to installed directory, find WMM.COF and remove it. 
Replace it with the new WMM.COF. 


WMM Linux C software
------------------------------------

For the version <= WMM2020, replace the WMM.COF file in the "bin" directory with the
new provided WMM.COF file. 

For the version >= WMM2025, WMM.COF is located at bin and data folder. No need to be replaced the WMM.COF. 

WMM Windows C software
------------------------------------
For the version <= WMM2020, replace the WMM.COF file in the "bin" directory with the
new provided WMM.COF file. 

For the version >= WMM2025, WMM.COF is in bin folder. No need to be replaced the WMM.COF. 


Your own software
-----------------
Depending on your installation, find the coefficient 
file, WMM.COF (unless renamed). Replace it with the new
WMM.COF file (renaming it appropriately if necessary).

If the coefficients are embedded in your software, you
may need to embed the new coefficients.

2. Installation Verification
============================

To confirm you are using the correct WMM.COF file open 
it and verify that the header is:

    2025.0            WMM-2025        11/13/2024

To assist in confirming that the installation of the new 
coefficient file is correct we provide a set of test 
values in this package.  Here are a few as an example:

Date     HAE   Lat   Long       X         Y         Z           H          F         Incl    Decl   GV        Xdot    Ydot    Zdot    Hdot    Fdot     Idot    Ddot
2025.0    0.0   80.0    0.0     6521.6      145.9    54791.5     6523.2    55178.5   83.21    1.28    1.28    -8.3    59.5    31.1    -7.0     30.1    0.01    0.52
2025.0    0.0  -80.0  240.0     6117.5    15751.9   -52022.5    16898.1    54698.2  -72.00   68.78  -51.22    33.3    -8.6    95.5     4.0    -89.6    0.03   -0.12
2027.5  100.0   80.0    0.0     6196.7      233.8    52670.5     6201.1    53034.3   83.29    2.16    2.16    -7.7    56.5    28.7    -5.6     27.8    0.01    0.52
2027.5  100.0  -80.0  240.0     5984.0    14760.1   -49317.7    15927.0    51825.7  -72.10   67.93  -52.07    30.6    -8.0    89.2     4.0    -83.7    0.03   -0.11
Where HAE is height above WGS-84 ellipsoid.




Model Software Support
======================

*  National Centers for Environmental Information (NCEI)
*  E/NE42 325 Broadway
*  Boulder, CO 80305 USA
*  Attn: Manoj Nair or Arnaud Chulliat
*  Phone:  (303) 497-4642 or -6522
*  Email:  geomag.models@noaa.gov
For more details about the World Magnetic Model visit 
http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml


