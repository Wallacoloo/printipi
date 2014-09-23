gparse
======

A simple (limited), dependency-free gcode parser for use in reprap-style 3d printers.

gparse was designed for the Printipi project (https://github.com/Wallacoloo/printipi). It takes input from a file-like object (.gcode file, stdin, or serial port) and parses each line into a Command object. 

gparse does not manage any of the state associated with gcode commands (eg unit modes or relative/absolute coordinates), rather it just parses the opcode and parameters for each command and allows one to return a response.

Limitations
======

There is currently almost no error-checking:  
Checksums are parsed, but not checked.  
Line numbers are ignored (and missing lines are not noticed).  
