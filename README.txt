Terra Height Field Simplification Software

This software is in the public domain and is provided AS IS. Use it at YOUR OWN RISK.

This is Terra, the successor to Scape. Not only is this software UNSUPPORTED, but it is in the early stages of development. This software is clearly incomplete and may still have bugs. However, the basic mechanisms do function properly; I have run several large terrains through Terra, and the results were correct.

For up-to-date information on Terra, Scape, and related topics, watch:

	http://www.cs.cmu.edu/~garland/scape/

Introduction

This is Terra, a program for generating polygonal approximations of terrains and other height fields. It is based on algorithms described in:

    Fast Polygonal Approximation of Terrains and Height Fields,
    by Michael Garland and Paul Heckbert (Technical Report CMU-CS-95-181).
    A color plate is included separately. 

The Scape package is the companion software for this paper. It was written with speed and memory efficiency as the primary concerns. Although it was designed strictly for the testing of the algorithms described in our paper, we made it available so that people interested in our results could examine it. We also hoped that it might be of interest to people who were attempting to build terrain approximations. However, Scape is not particularly easy to use and the code is definitely less than aesthetically pleasing.

I wrote Terra because I was dissatisfied with Scape. I wanted code which was better structured and programs which were easier to use. Terra will also provide more features than Scape.

Disclaimer: Please remember that both Terra and Scape are unsupported programs that I tinker with in my spare time. As such, development is typically sporadic.
Brief feature summary

PGM input files
    Terra uses the PGM file format for data input. At first, this might seem odd; however, it has several advantages. PGM is a standard format which provides for both textual (i.e., human editable) and binary data files. Since PGM is an image file format, height field data is directly viewable with most available image viewers. Plus, there are many programs available to perform various manipulations on PGM files.

    In particular, many of the tools provided by the NetPBM package can be used to manipulate PGM terrain data. The NetPBM package can be found online at:

    	http://wuarchive.wustl.edu/graphics/graphics/packages/NetPBM/

    or by anonymous ftp to:

    	wuarchive.wustl.edu:/graphics/graphics/packages/NetPBM

Flexible output
    Terra can output its approximations in several different formats. The supported formats are described below.

Extended approximation features
    Terra supports preinsertion scripts and importance masks. See below for details on what these are and how they work.

Portable graphics
    The interactive program, xterra, uses the GLUT library for windowing and OpenGL for rendering. In theory, this should make it portable to machines other than SGI's. 

Currently absent features

All these features are currently missing. Ideally, I would like to include these and other features. In reality, what gets done and how fast it gets done might be highly variable.

Multiple simultaneous height fields
    Scape provided for approximating height fields with RGB textures applied to them. Ultimately, Terra will support the approximation of arbitrarily many simultaneous height fields. Currently, however, Terra will only accept the input of a single height field.

Data-dependent triangulation
    One of the significant features of Scape was the option to use data-dependent triangulation. This triangulation scheme has not yet been ported to Terra.

Data import facilities
    I would like to be able to import USGS DEM data into Terra. Although correcting for the spherical mapping of USGS data is beyond the scope of this project, Terra is in need of some simple tools to facilitate conversion of USGS data. I definitely don't have the time to write these tools, so if someone wants to suggest some reasonable ones, please let me know.

Robust PGM input routines
    The PGM input routines are rather fragile at present. You should make sure that there is no extraneous white space and no comments in the file. For example, the xv program will insert a comment in the PGM file citing itself as the creator of the file. You will need to remove this comment from the file. 

Installation

    In order to compile the interactive version of Terra (xterra), you must obtain the GLUT library. It can be found at:

    	http://www.sgi.com/Technology/openGL/glut.html

    or by anonymous ftp at:

    	sgigate.sgi.com:/pub/opengl/xjournal/GLUT

    NOTE: For proper viewing, xterra needs to set the aspect ratio of its windows. This is currently not supported via GLUT. Therefore, I've had to hack things a bit by accessing GLUT internals. The file gui.cc includes the glutint.h header. This is not installed by GLUT. You must install this manually. Again, this is only of importance if you want to build xterra.

    Edit the Makefile for local customization. Basically, you should set the compilation flags, the libraries you need, and the location of GLUT if you are compiling xterra.

    Typing 'make' will build both terra and xterra. However, you can selectively compile either of them (e.g., 'make terra'). 

Using Terra

The Terra software provides two programs: terra, a simple batch program, and xterra, an interactive application which uses OpenGL to display the surface approximation being constructed. Both programs utilize the same command line interface; xterra simply layers an interactive interface on top of terra. This section contains an outline of the operation of terra. All this information applies to xterra as well.

The operation of Terra goes through a simple series of phases:

    Data input.
    This will load the terrain data and the importance mask (if specified).
    Pre-insertion.
    If you have specified a pre-insertion script, it will be executed at this point.
    Greedy insertion.
    The iterative greedy insertion procedure will begin. Terra will continue selecting points until it meets the goals that you have specified.
    Output.
    Terra will output the approximation it has constructed. You have a choice of a handful of different formats. 

Currently, all the information Terra needs to run is communicated on the command line. The correct usage of Terra is:

	usage: terra <options> filename
	       where <options> is some combination of: 
	-e <thresh>      Sets the tolerable error threshold
	-p <count>       Sets the maximum number of allowable points
	-o <file> <type> When finished, output the approximation to <file>.
	                 Valid types:  tin, eps, dem, obj
	-m <file>        Load the importance mask from <file>
	-s <file>        Execute preinsertion script from <file>

The error threshold and point limit set the termination criteria. Terra will continue adding points as long as it it below the point limit and above the error threshold. The default error threshold is 0; the default point limit is the total number of points in the input grid.

The type of output desired is also specified on the command line. The eps output format simply generates an Encapsulated PostScript rendering of the approximation mesh. The dem output format creates an approximate DEM from the approximate mesh. This can be useful for comparison with the original. Both the tin and obj output formats generate 3D surfaces. The obj format is just the Wavefront .OBJ file format. The tin format is a very simple model description; it is a series of lines where each line is of the form:

	t x1 y1 z1 x2 y2 z2 x3 y3 z3

Each such line describes a triangle with the three corners (x1,y1,z1) (x2,y2,z2) (x3,y3,z3).

The remaining options, involving importance masks and preinsertion scripts, are described in detail below.
Xterra User Interface

xterra accepts the same options as terra and operates in much the same way. It introduces one extra command line option:

	-h <factor>      Sets the height scaling factor.  For example,
	                 if grid points are 25m apart, use a factor of 0.04

This is used to properly scale the display of the height field in 3D. The input to Terra is specified in pixel coordinates; data samples are taken at integral pixel coordinates. However, the height values are probably not given in pixel coordinates. So, for display purposes, the height values are scaled by a constant factor to account for this loss of units in Terra.

When xterra starts up, it performs any preinsertion actions that you request, and then it displays two windows: a mesh view and a surface view. It does not perform greedy insertion until told to do so. The mesh view provides a 2D view of the triangulation being generated for the approximation of the height field. The surface view displays the approximate surface in 3D. The interaction with these windows is currently quite simple and will probably change in the future, but here is an outline of how they work at the moment:

	Surface view:
	   Left mouse drag   :   spin the surface
	   Middle mouse drag :   translate the camera side to side
	   Right mouse drag  :   move the camera in and out

	Mesh view:
	   Left mouse click   :  insert a point at the mouse location
	   Middle mouse click :  run greedy insertion until goal is met
	   Right mouse click  :  popup menu -- allows output

Providing Input for Terra

As stated above, Terra uses PGM files to read and write height field data. Unfortunately, Terra does not as yet provide any direct means of acquiring PGM data. For now, you will have to use the conversion software provided with Scape. The STM-tools/stm2pgm utility distributed with Scape can convert Scape's STM file format into PGM's appropriate for use with Terra. Given an STM file,

	stm2pgm sample.stm exact > sample.pgm

will generate a PGM file. Note that the keyword exact is very important. Don't forget it! The resulting file will be textual, so you can even edit by hand if you need to.
Importance Masks

One of the new features in Terra not found in Scape is the use of importance masks. In order to determine the next point for insertion, Terra ranks the available points by an importance measure. Importance masks allow you to bias this ranking. For each data point, the mask assigns a weight in the range [0..1] which multiplies the computed importance value.

Importance masks are PGM files, just like the height field input. However, their interpretation is slightly different. The input values are all integers. They are scaled such that their maximum value will be mapped to 1. One significant point is that this maximum value is taken from the PGM header, not determined from the data. Therefore, by controlling the stated "maximum", you have much greater flexibility over the mapping of PGM pixel values to importance mask weights.

Currently, no means for constructing importance masks is provided.
Preinsertion Scripts

The other new feature of Terra is its support for preinsertion scripts. An important feature of the greedy insertion algorithm is that essentially provides for progressive refinement of the approximation. Thus, the initial approximation can be arbitrary. The preinsertion scripts allow you set up an approximation before greedy insertion begins.

A preinsertion script is a series of lines, each of the form:

	<op> X Y

The values for X and Y are the coordinates of a particular data point. The currently supported opcodes are:

	s -- Select this point for use in the approximation and insert
	     it into the current mesh.

	i -- Mark this point as one to be ignored.  Terra will never
	     process this point for insertion or evaluation.

	u -- Mark the height value at this point as unknown.

Currently, Terra makes no distinction between points to be ignored and points whose height value is unknown; it ignores them equally.

January 19, 1996
Michael Garland
garland@cs.cmu.edu




*******************************************************************
* SCAPE Terrain Simplification Software                           *
* Michael Garland and Paul Heckbert, 1994.                        *
*                                                                 *
* This software is public domain and is provided AS IS.           *
* Use it at YOUR OWN RISK.                                        *
*******************************************************************

This software is UNSUPPORTED.  We do not have the time to actively
support this software.  However, we would be happy to collect any
improvements made by others.  We have tried to make this code
palatable, but it is a long way from aesthetically pleasing.

Hopefully, you find this software interesting or useful.  If you do,
we would appreciate hearing from you.  Please send any improvements,
comments, or glowing testimonials to:

		garland@cs.cmu.edu

The algorithms used in this software are described in our technical
report on fast approximation of height fields.  You can find a copy of
this paper at:

	http://www.cs.cmu.edu/afs/cs/user/garland/public/scape/index.html


IMPORTANT NOTE: scape is now even less supported.  It will be replaced
by a new and improved program: terra.  Check the Web page above for
details.

------------------------------------------------------------------------

The programs:

	scape - The basic program.  Generates a TIN approximation for
                a given height field.  Creates a file 'out.tin'
		containing the approximation.

	glscape   - Interactive terrain simplification (SGI only).
	drawscape - Just draws an STM model (SGI only).

	Samples/ - contains some sample height fields.

[Invoke the programs without arguments to see the available arguments]

------------------------------------------------------------------------

The SCAPE tools use the STM (Simple Terrain Model) file format.  It
consists of a brief header followed by raw binary data.  The general
structure of the file is:


	STM <width> <height> <c1><c2><c3><c4><cr>
	<height data>

The height data is a sequence of unsigned 16-bit integers representing
the height values.

The four bytes c1..c4 are used to determine what byte order the
integers are stored in.

The STM-tools directory contains some simple programs for creating and
manipulating STM files.

In particular, the DEM2STM script converts USGS DEM files into STM
files.  It uses the CONVERT program by Christopher Keane to parse the
DEM file into a simpler format, and then it converts that into an STM
file.  This will NOT account for the spherical mapping of DEM data.
To properly warp the input data, look at the demtoflat and resample
programs in the STM-tools/DEMutil directory.

*PLEASE NOTE*  These converters are in no way meant to be highly
accurate or production quality.  They are merely included as quick and
dirty examples to let people experiment with USGS DEM data.

------------------------------------------------------------------------

The 'glscape' program allows you to watch the process of terrain
simplification.  The keys for performing operations are:

Mesh view (the initial window):

	g - select multiple sites (number specified on command line)
	s - select a single site
	r - bring up surface view
	q - quit scape program

Surface view:

	left button drag - rotate terrain around
	middle button drag - move light when in lit-surface mode

	t - toggle display of triangle mesh
	f - toggle display between pseudocolor and lit-surface modes
	v - toggle display of surface texture
	x - show actual surface texture rather than approximate texture

	g - select multiple sites
	s - select a single site
	q - quit surface view

------------------------------------------------------------------------

The SCAPE system is based on algorithms described in our paper:

	"Fast Polygonal Approximation of Terrains and Height Fields"
	by Michael Garland and Paul Heckbert
	(Technical Report CMU-CS-95-181)

The results shown in that paper were generated by the SCAPE software.
However, there are some differences between this software and what is
described in the paper.

	(1) The unoptimized algorithms which we discuss are not
	supported by SCAPE.  Only the optimized algorithms
	(III and IV from the paper) are implemented here.

	(2) Memory usage for the heap in SCAPE is a bit different.  In
	the paper, we say that the memory necessary for the heap is
	proportional to the number of triangles.  This is not strictly
	true in SCAPE.  To avoid large memory copies during the
	simplification process, SCAPE preallocates a very large heap.
	SCAPE will only actually use the amount (as given in the
	paper) which is proportional to the number of triangles.  On
	systems which support virtual memory, this does not incur any
	penalty: the unused portions of the heap are never touched and
	remain either unallocated or paged out on the disk.
