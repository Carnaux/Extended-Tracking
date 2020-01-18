// ============================================================================
//	Includes
// ============================================================================


//====ORB-SLAM2 includes=====

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

// ============================



#ifdef _WIN32
#  include <windows.h>
#endif
#include <stdio.h>
#ifdef _WIN32
#  define snprintf _snprintf
#endif
#include <string.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif


#include <AR/ar.h>
#include <AR/arMulti.h>
#include <AR/video.h>
#include <AR/gsub_lite.h>
#include <AR/arFilterTransMat.h>
#include <AR2/tracking.h>

#include "ARMarkerNFT.h"
#include "trackingSub.h"
#include <KPM/kpm.h>



// ============================================================================
//	Constants
// ============================================================================

#define PAGES_MAX               10       // Maximum number of pages expected. You can change this down (to save memory) or up (to accomodate more pages.)

#define VIEW_SCALEFACTOR		1.0		// Units received from ARToolKit tracking will be multiplied by this factor before being used in OpenGL drawing.
#define VIEW_DISTANCE_MIN		10.0		// Objects closer to the camera than this will not be displayed. OpenGL units.
#define VIEW_DISTANCE_MAX		10000.0		// Objects further away from the camera than this will not be displayed. OpenGL units.

// ============================================================================
//	Global variables
// ============================================================================


// ===============ORB-SLAM2=================

    ORB_SLAM2::System *ptr_SLAM;
    ORB_SLAM2::System::my_eTrackingState *ptr_State; // Used to mark the status of the SLAM system
    
// =========================================


// Preferences.
static int prefWindowed = TRUE;
static int prefWidth = 640;				// Fullscreen mode width.
static int prefHeight = 480;				// Fullscreen mode height.
static int prefDepth = 32;				// Fullscreen mode bit depth.
static int prefRefresh = 0;				// Fullscreen mode refresh rate. Set to 0 to use default rate.


// Image acquisition.
static ARUint8		*gARTImage = NULL;		// Used to save the acquired frame of pictures
static long		gCallCountMarkerDetect = 0;	// Used to record the total number of frames that have been obtained so far when the program is running



// Markers.
ARMarkerNFT *markersNFT = NULL;			// (ARMarkerNFT array)-used to save the state of the nft marker
int markersNFTCount = 0;				// Saved in this program, indicating that there are several nft targets to be tracked

// NFT.
static THREAD_HANDLE_T     *threadHandle = NULL;	// Set when nft is loaded (set when the KPM thread is started), save the relevant information of the thread, as the parameter of the trackingInitMain function
static AR2HandleT          *ar2Handle = NULL;		// Save the current state of the nft texture tracker, mainly some setting information during the tracking process
static KpmHandle           *kpmHandle = NULL;		// The structure used for the feature matching function kpmMatching (), which saves the parameters (including the data set) required by kpm tracking and the results of kpm tracking
static int                  surfaceSetCount = 0;	// Save the number of nft tags in this project, set in loadnft
static AR2SurfaceSetT      *surfaceSet[PAGES_MAX];	// Used to save .fset data set information, each nft marker is stored separately for AR2tracking




// Drawing.
static int gWindowW;
static int gWindowH;
static ARParamLT *gCparamLT = NULL;			// Save the camera's internal parameters
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;	// Set this parameter after building the argl library for the OpenGL environment
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 0;			// For use in drawing.
static ARdouble cameraLens[16];				// Required when creating OpenGL projection matrix through internal parameters, the role is unknown



// ============================================================================
//	Function prototypes
// ============================================================================


static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p);
static int initNFT(ARParamLT *cparamLT, AR_PIXEL_FORMAT pixFormat);
static int loadNFTData(void);
static void cleanup(void);
static void Keyboard(unsigned char key, int x, int y);
static void Visibility(int visible);
static void Reshape(int w, int h);
static void Display(void);



// ============================================================================
//	Functions
// ============================================================================


int main(int argc, char** argv)
{
    char glutGamemode[32];
    const char *cparam_name = "Data2/camera_para.dat";			// Used to save the camera parameter file path, read the camera reference
    char vconf[] = "";
    const char markerConfigDataFilename[] = "Data2/markers.dat";	// Used to save the mark file path
	
    
#ifdef DEBUG
    arLogLevel = AR_LOG_LEVEL_DEBUG;
#endif
// ==============================ORB-SLAM2======================================
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary / path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.      // Initialize all threads
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,&ptr_State,true);  // Dictionary file, internal reference file SLAM is a system variable

    ptr_SLAM = &SLAM;

// =============================================================================


    //
	// Library inits.
	//
    
	glutInit(&argc, argv);		// Init the glut library
    
	//
	// Video setup.
	//
    
#ifdef _WIN32
	CoInitialize(NULL);
#endif
    
	if (!setupCamera(cparam_name, vconf, &gCparamLT)) {	// Open the camera, read the camera internal parameters, and save it to gCparamLT
		ARLOGe("main(): Unable to set up AR camera.\n");
		exit(-1);
	}
	
	
    //
    // AR init.
    //
    
    // Create the OpenGL projection from the calibrated camera parameters.
    arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, cameraLens);	// Build OpenGL projection matrix based on camera internal parameters
    
    
    
    if (!initNFT(gCparamLT, arVideoGetPixelFormat())) { // Initiate nft initialization work, mainly to allocate and initialize the structures kpmHandle and ar2Handle needed by KPM tracking and ar2tracking
		ARLOGe("main(): Unable to init NFT.\n");
		exit(-1);
    }

	//
	// Graphics setup.
	//
    
	// Set up GL context(s) for OpenGL to draw into.	// Set openGL initialization display mode
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	
	if (!prefWindowed) {
		if (prefRefresh) sprintf(glutGamemode, "%ix%i:%i@%i", prefWidth, prefHeight, prefDepth, prefRefresh);
		else sprintf(glutGamemode, "%ix%i:%i", prefWidth, prefHeight, prefDepth);
		glutGameModeString(glutGamemode);
		glutEnterGameMode();
	} else {
		glutInitWindowSize(gCparamLT->param.xsize, gCparamLT->param.ysize);
		glutCreateWindow(argv[0]);
	}
    
	// Setup ARgsub_lite library for current OpenGL context.	// Build the argl library for the current OpenGL environment
	if ((gArglSettings = arglSetupForCurrentContext(&(gCparamLT->param), arVideoGetPixelFormat())) == NULL) {
		ARLOGe("main(): arglSetupForCurrentContext() returned error.\n");
		cleanup();
		exit(-1);
	}
	
	arUtilTimerReset();	// At the beginning, that is, when the number of frames is 0, reset the timer arUtilTimerReset, so as to use arUtilTimer to get the time consumed from reset
    
    //
    // Markers setup.
    //
    
	
    // Load marker(s).
    newMarkers(markerConfigDataFilename, &markersNFT, &markersNFTCount);// Load marker file information and set markersNFT and markersNFTCount, where "valid validPrev ftmi filterCutoffFrequency filterSampleRate pageNo datasetPathname is set" in markersNFT
    if (!markersNFTCount) {
        ARLOGe("Error loading markers from config. file '%s'.\n", markerConfigDataFilename);
		cleanup();
		exit(-1);
    }
    
    ARLOGi("Marker count = %d\n", markersNFTCount);
    
    
    // Marker data has been loaded, so now load NFT data.	// Load the nft data set (natural image markup) and set kpmHandle-> refDataSet
    if (!loadNFTData()) {
        ARLOGe("Error loading NFT data.\n");
		cleanup();
		exit(-1);
    }    
    
    // Start the video.
    if (arVideoCapStart() != 0) {				// Open the camera and start capturing video frames
    	ARLOGe("setupCamera(): Unable to begin camera data capture.\n");
		return (FALSE);
	}
	
	// Register GLUT event-handling callbacks.
	// NB: mainLoop() is registered by Visibility.
	
	
	// Set each callback event of glut, and then use glutMainLoop () to make each callback event execute
	
	glutDisplayFunc(Display);		// When the window needs to be redrawn, execute Display
	glutReshapeFunc(Reshape);		// When changing the window size, execute Reshape
	glutVisibilityFunc(Visibility);		// When the glut window is displayed, execute visibility (the first time it is displayed, it will be called once)
	
	glutKeyboardFunc(Keyboard);		// Handle keyboard input events
	
	glutMainLoop();

	return (0);
}

// Something to look at, draw a rotating colour cube.
static void DrawCube(void)
{
    // Colour cube data.
    int i;
	float fSize = 40.0f;
    const GLfloat cube_vertices [8][3] = {
        /* +z */ {0.5f, 0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f},
        /* -z */ {0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f} };
    const GLubyte cube_vertex_colors [8][4] = {
        {255, 255, 255, 255}, {255, 255, 0, 255}, {0, 255, 0, 255}, {0, 255, 255, 255},
        {255, 0, 255, 255}, {255, 0, 0, 255}, {0, 0, 0, 255}, {0, 0, 255, 255} };
    const GLubyte cube_faces [6][4] = { /* ccw-winding */
        /* +z */ {3, 2, 1, 0}, /* -y */ {2, 3, 7, 6}, /* +y */ {0, 1, 5, 4},
        /* -x */ {3, 0, 4, 7}, /* +x */ {1, 2, 6, 5}, /* -z */ {4, 5, 6, 7} };
    
    glPushMatrix(); // Save world coordinate system.
    glRotatef(gDrawRotateAngle, 0.0f, 0.0f, 1.0f); // Rotate about z axis.
    glScalef(fSize, fSize, fSize);
    glTranslatef(0.0f, 0.0f, 0.5f); // Place base of cube on marker surface.
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, cube_vertex_colors);
    glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    for (i = 0; i < 6; i++) {
        glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    }
    glDisableClientState(GL_COLOR_ARRAY);
    glColor4ub(0, 0, 0, 255);
    for (i = 0; i < 6; i++) {
        glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    }
    glPopMatrix();    // Restore world coordinate system.
}

static void DrawCubeUpdate(float timeDelta)
{
	if (gDrawRotate) {
		gDrawRotateAngle += timeDelta * 45.0f; // Rotate cube at 45 degrees per second.
		if (gDrawRotateAngle > 360.0f) gDrawRotateAngle -= 360.0f;
	}
}

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p)
{	
    ARParam			cparam;				// Used to save camera internal parameters
    int				xsize, ysize;		// Save the size of the video window (width, height)
    AR_PIXEL_FORMAT pixFormat;          // Save pixel format

    
    // Open the video path.					//Turn on the camera
    if (arVideoOpen(vconf) < 0) {			// What is the last opened camera (including video window size) related to vconf
    	ARLOGe("setupCamera(): Unable to open connection to camera.\n");
    	return (FALSE);
	}
	
    // Find the size of the window.
    if (arVideoGetSize(&xsize, &ysize) < 0) {	// Get the size of the video frame (the actual video window)
        ARLOGe("setupCamera(): Unable to determine camera frame size.\n");
        arVideoClose();
        return (FALSE);
    }
    ARLOGi("Camera image size (x,y) = (%d,%d)\n", xsize, ysize);
	
	// Get the format in which the camera is returning pixels.
	pixFormat = arVideoGetPixelFormat();	// Get the picture pixel format of the video frame: AR_PIXEL_FORMAT_BGR
	if (pixFormat == AR_PIXEL_FORMAT_INVALID) {
    	ARLOGe("setupCamera(): Camera is using unsupported pixel format.\n");
        arVideoClose();
		return (FALSE);
	}
	
	    //ARLOGi("pixFormat: %d\n",pixFormat);		// After testing, pixFormat = AR_PIXEL_FORMAT_BGR
	  
	// Load the camera parameters, resize for the window and init.
    if (arParamLoad(cparam_name, 1, &cparam) < 0) {		// Read camera internal parameters (video frame size, projection matrix P, lens distortion parameters) and save to cparam
		ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
        arVideoClose();
        return (FALSE);
    }
    
    
    // Test camera internal parameters:
    cout<<"矩阵："<<endl;
    for(int i=0;i<3;i++)
    {
      for(int j=0;j<4;j++)
	cout<<cparam.mat[i][j]<<"  ";
      cout<<endl;
    }
    
    cout<<"dist_factor:"<<endl;
    for(int i=0;i<AR_DIST_FACTOR_NUM_MAX;i++)
    cout<<cparam.dist_factor[i]<<endl;
    
    if (cparam.xsize != xsize || cparam.ysize != ysize) {	// Modify the xsize and ysize in cparam to the actual video frame size
        ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
        arParamChangeSize(&cparam, xsize, ysize, &cparam);
    }
    
#ifdef DEBUG
    ARLOG("*** Camera Parameter ***\n");
    arParamDisp(&cparam);
#endif
    
    
    // Construct the structure ARParamLT object according to the cparam information, and make the global variable gCparamLT (ARParamLT *) point to it
    if ((*cparamLT_p = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {	
        ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
        arVideoClose();
        return (FALSE);
    }
	
	return (TRUE);
}


// Modifies globals: kpmHandle, ar2Handle.

static int initNFT(ARParamLT *cparamLT, AR_PIXEL_FORMAT pixFormat)
{
    ARLOGd("Initialising NFT.\n");
    //
    // NFT init.
    //
    
    // KPM init.
    kpmHandle = kpmCreateHandle(cparamLT, pixFormat);	// For the matching function kpmMatching ()-KPM tracking allocates and initializes the kpmHandle structure, which mainly sets the cparamLT, poseMode, xsize, ysize, pixFormat, procMode, and no data set information is set yet
    if (!kpmHandle) {
        ARLOGe("Error: kpmCreateHandle.\n");
        return (FALSE);
    }
    //kpmSetProcMode( kpmHandle, KpmProcHalfSize );
    
    
    // AR2 init.
    if( (ar2Handle = ar2CreateHandle(cparamLT, pixFormat, AR2_TRACKING_DEFAULT_THREAD_NUM)) == NULL ) {
        ARLOGe("Error: ar2CreateHandle.\n");		// Initiate ar2Handle structure for ar2tracking, and create multiple ar2tracking threads to wait for processing
        kpmDeleteHandle(&kpmHandle);
        return (FALSE);
    }
    
    if (threadGetCPU() <= 1) {
        ARLOGi("Using NFT tracking settings for a single CPU.\n");
        ar2SetTrackingThresh(ar2Handle, 5.0);
        ar2SetSimThresh(ar2Handle, 0.50);
        ar2SetSearchFeatureNum(ar2Handle, 16);
        ar2SetSearchSize(ar2Handle, 6);
        ar2SetTemplateSize1(ar2Handle, 6);
        ar2SetTemplateSize2(ar2Handle, 6);
    } else {								// has 4 tracking threads
        ARLOGi("Using NFT tracking settings for more than one CPU.\n");
        ar2SetTrackingThresh(ar2Handle, 5.0);			// Set pose estimation threshold
        ar2SetSimThresh(ar2Handle, 0.50);
        ar2SetSearchFeatureNum(ar2Handle, 16);
        ar2SetSearchSize(ar2Handle, 12);			// Set the feature point search radius
        ar2SetTemplateSize1(ar2Handle, 6);
        ar2SetTemplateSize2(ar2Handle, 6);
    }
    // NFT dataset loading will happen later.
    return (TRUE);
}

// Modifies globals: threadHandle, surfaceSet[], surfaceSetCount
static int unloadNFTData(void)
{
    int i, j;
    
    if (threadHandle) {
        ARLOGi("Stopping NFT2 tracking thread.\n");
        trackingInitQuit(&threadHandle);
    }
    j = 0;
    for (i = 0; i < surfaceSetCount; i++) {
        if (j == 0) ARLOGi("Unloading NFT tracking surfaces.\n");
        ar2FreeSurfaceSet(&surfaceSet[i]); // Also sets surfaceSet[i] to NULL.
        j++;
    }
    if (j > 0) ARLOGi("Unloaded %d NFT tracking surfaces.\n", j);
    surfaceSetCount = 0;
    
    return 0;
}

// References globals: markersNFTCount
// Modifies globals: threadHandle, surfaceSet[], surfaceSetCount, markersNFT[]

static int loadNFTData(void)		// Load the nft dataset for ar2tracking and kpmtracking, and start the kpm thread
{
    int i;
    KpmRefDataSet *refDataSet;
    
    // If data was already loaded, stop KPM tracking thread and unload previously loaded data.
    if (threadHandle) {
        ARLOGi("Reloading NFT data.\n");
        unloadNFTData();
    } else {	//该条语句执行
        ARLOGi("Loading NFT data.\n");
    }
    
    refDataSet = NULL;
    
    for (i = 0; i < markersNFTCount; i++) {
      
        // Load KPM data.
        KpmRefDataSet  *refDataSet2;
	
        ARLOGi("Reading %s.fset3\n", markersNFT[i].datasetPathname);	// Read the file pinball.fset3
	
        if (kpmLoadRefDataSet(markersNFT[i].datasetPathname, "fset3", &refDataSet2) < 0 ) {	// Load the data in pinball.fset3 to refDataSet2
            ARLOGe("Error reading KPM data from %s.fset3\n", markersNFT[i].datasetPathname);
            markersNFT[i].pageNo = -1;
            continue;
        }
        
        markersNFT[i].pageNo = surfaceSetCount;			// Number each marker separately
        
        ARLOGi("  Assigned page no. %d.\n", surfaceSetCount);
	
        if (kpmChangePageNoOfRefDataSet(refDataSet2, KpmChangePageNoAllPages, surfaceSetCount) < 0) {
            ARLOGe("Error: kpmChangePageNoOfRefDataSet\n");
            exit(-1);
        }
        if (kpmMergeRefDataSet(&refDataSet, &refDataSet2) < 0) {	// Merge data set information into refDataSet and destroy refDataSet2
            ARLOGe("Error: kpmMergeRefDataSet\n");
            exit(-1);
        }
        ARLOGi("  Done.\n");
        
	
        // Load AR2 data.
        ARLOGi("Reading %s.fset\n", markersNFT[i].datasetPathname);	// Read the file pinball.fset
        
        if ((surfaceSet[surfaceSetCount] = ar2ReadSurfaceSet(markersNFT[i].datasetPathname, "fset", NULL)) == NULL ) {
            ARLOGe("Error reading data from %s.fset\n", markersNFT[i].datasetPathname);
        }					// Put the information read from pinball.fset to surfaceSet
        ARLOGi("  Done.\n");
        
        surfaceSetCount++;
        if (surfaceSetCount == PAGES_MAX) break;
    }
     
    // Set the dataset to the dataset currently tracked by kpm
    if (kpmSetRefDataSet(kpmHandle, refDataSet) < 0) {	// Update refDataSet to kpmHandle-> refDataSet
        ARLOGe("Error: kpmSetRefDataSet\n");
        exit(-1);
    }
    kpmDeleteRefDataSet(&refDataSet);
    
    
    // Start the KPM tracking thread.
    threadHandle = trackingInitInit(kpmHandle);		// Start KPM thread
    if (!threadHandle) exit(-1);

	ARLOGi("Loading of NFT data complete.\n");
    return (TRUE);
}

static void cleanup(void)
{
    if (markersNFT) deleteMarkers(&markersNFT, &markersNFTCount);
    
    // NFT cleanup.
    unloadNFTData();
    ARLOGd("Cleaning up ARToolKit NFT handles.\n");
    ar2DeleteHandle(&ar2Handle);
    kpmDeleteHandle(&kpmHandle);
    arParamLTFree(&gCparamLT);

    // OpenGL cleanup.
    arglCleanup(gArglSettings);
    gArglSettings = NULL;
    
    // Camera cleanup.
	arVideoCapStop();
	arVideoClose();
#ifdef _WIN32
	CoUninitialize();
#endif
}

static void Keyboard(unsigned char key, int x, int y)
{
	ARLOGi("keyboard\n");
	switch (key) {
		case 0x1B:						// Quit.
		case 'Q':
		case 'q':
			cleanup();
			exit(0);
			break;
		case ' ':
			gDrawRotate = !gDrawRotate;
			break;
		case '?':
		case '/':
			ARLOG("Keys:\n");
			ARLOG(" q or [esc]    Quit demo.\n");
			ARLOG(" ? or /        Show this help.\n");
			ARLOG("\nAdditionally, the ARVideo library supplied the following help text:\n");
			arVideoDispOption();
			break;
		default:
			break;
	}
}

static void mainLoop(void)
{
	static int ms_prev;
	int ms;
	float s_elapsed;
	ARUint8 *image;

    // NFT results.
    static int detectedPage = -2; // -2 Tracking not initied, -1 tracking initied OK, >= 0 tracking online on page.
    static float trackingTrans[3][4];	// Used to store the transformation matrix of the template coordinate system to the camera coordinate system
    

    int             i, j, k;


	// Find out how long since mainLoop() last ran.		// Calculate the execution interval of the two mainloops
	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	if (s_elapsed < 0.01f) return; // Don't update more often than 100 Hz.	// If the two times are less than 0.01s, return (make the refresh frequency lower than 100hz)
	ms_prev = ms;
	
	
	// Update drawing.		
	DrawCubeUpdate(s_elapsed);		// When you enter space on the keyboard, the statement will be executed
	
	
	// Grab a video frame.
	if ((image = arVideoGetImage()) != NULL) {	// Call the function to get a frame of picture
		gARTImage = image;	// Save the fetched image.
		gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

        // Run marker detection on frame
        if (threadHandle) {	// kpm's threadhandle
            // Perform NFT tracking.
            float            err;
            int              ret;
            int              pageNo;
            
            if( detectedPage == -2 ) {		// Pass the newly acquired frame to threadHandle-> trackingInitHandle-> imageSize, and send a start signal to kpmtracking
                trackingInitStart( threadHandle, gARTImage );
                detectedPage = -1;		// Set detectedPage = -1 to indicate that the picture has been passed to kpmtracking, and kpmtracking has begun to work
            }
            
            if( detectedPage == -1 ) {		// kpmtracking has started working
	      
                ret = trackingInitGetResult( threadHandle, trackingTrans, &pageNo);	// Try to get the result of kpmtracking-that is, the pose of the camera. If successful, it returns 1; otherwise it returns -1.
		
                if( ret == 1 ) {	// Successfully obtained the camera pose from kpmtracking
                    if (pageNo >= 0 && pageNo < surfaceSetCount) {// If the returned camera pose is correct, then transfer the subsequent pose acquisition from kpmtracking to ar2tracking
                        ARLOGi("Detected page %d.\n", pageNo);
			
                        detectedPage = pageNo;			// Set detectedPage for subsequent ar2Tracking
                        
                        ar2SetInitTrans(surfaceSet[detectedPage], trackingTrans); // Put the pose results obtained from kpmtracking into surfaceSet-> trans1, and set surfaceSet-> contNum = 1, surfaceSet-> prevFeature [0] .flag = -1
                    } else {
                        ARLOGi("Detected bad page %d.\n", pageNo);
                        detectedPage = -2;
                    }
                } else if( ret < 0 ) {		// No pose detected
                    ARLOGi("No page detected.\n");
                    detectedPage = -2;
                }
            }
            
	     // After detecting the template successfully, start here directly every time
            if( detectedPage >= 0 && detectedPage < surfaceSetCount) {// When kpmtracking detects the marker and obtains the pose, use this value to initialize ar2tracking and submit it to ar2tracking to obtain the subsequent pose
                if( ar2Tracking(ar2Handle, surfaceSet[detectedPage], gARTImage, trackingTrans, &err) < 0 ) {	// The function is similar to KPMtracking
                    ARLOGd("Tracking lost.\n");	 // Tracking is lost
                    detectedPage = -2;
                } else {
                    ARLOGi("Tracked page %d (max %d).\n", detectedPage, surfaceSetCount - 1);		// Successful tracking
                }
            }
            
        } else {
            ARLOGe("Error: threadHandle\n");
            detectedPage = -2;
        }
        
        // Update markers.
        for (i = 0; i < markersNFTCount; i++) {
	  
            markersNFT[i].validPrev = markersNFT[i].valid;
	    
            if (markersNFT[i].pageNo >= 0 && markersNFT[i].pageNo == detectedPage) {	// ar2tracking tracking success
                markersNFT[i].valid = TRUE;
		
                for (j = 0; j < 3; j++){
                    for (k = 0; k < 4; k++){
                        markersNFT[i].trans[j][k] = trackingTrans[j][k];  // If ar2tracking is successful, set makersNFT[i].trans to the result of ar2tracking
                    }
                } 	
	        }
            else{ 
                markersNFT[i].valid = FALSE; // This time tracking failed
            }	
	    
            if (markersNFT[i].valid) {	// this time success

                // Filter the pose estimate.
		
                if (markersNFT[i].ftmi) {
                    if (arFilterTransMat(markersNFT[i].ftmi, markersNFT[i].trans, !markersNFT[i].validPrev) < 0) {	// Perform filtering optimization for pose results
                        ARLOGe("arFilterTransMat error with marker %d.\n", i);
                    }
                }
 
		
                // ====================ORB-SLAM2============================

                ARfloat   trans_temp[4][4];
                for(int m=0;m<3;m++)
                for(int n=0;n<4;n++)
                    trans_temp[m][n]=markersNFT[i].trans[m][n];
                
                trans_temp[3][0]=trans_temp[3][1]=trans_temp[3][2]=0;
                trans_temp[3][3]=1;    
                cv::Mat Tcm(4,4,CV_32F,(ARfloat *)trans_temp);	// Get the transformation matrix of the template coordinate system to the camera coordinate system		
                
                cv::Mat im(480,640,CV_8UC3,gARTImage);	// Construct mat type image
            
                
                if(im.empty()){
                    cerr << endl << "Failed to load image from ARToolkit."<<endl;
                    return;
                }
                
                // Get the current timestamp
                std::chrono::system_clock::time_point t_epoch;
                std::chrono::duration<double,std::ratio<1,1000000>> duration_micr_sec = std::chrono::system_clock::now() - t_epoch;
                double tframe = duration_micr_sec.count() / 1000000.0;
                
                (*ptr_SLAM).TrackMonocular(im,tframe,Tcm,1); // Import pictures and timestamps and poses into the SLAM system
                
                // =========================================================
                
                if (!markersNFT[i].validPrev) {		
                    // Last time failed? ——Marker never recognized
                    // Marker has become visible, tell any dependent objects.
                    // --->
                }
                
                // We have a new pose, so set that.
                arglCameraViewRH((const ARdouble (*)[4])markersNFT[i].trans, markersNFT[i].pose.T, VIEW_SCALEFACTOR);
                // Tell any dependent objects about the update.
                // --->
                
            } else{	// Fail this time, try to get the calculated pose from ORB-SLAM in the case of ARToolkit tracking failure
                if((*ptr_State)==ORB_SLAM2::System::OK || (*ptr_State)==ORB_SLAM2::System::LOST)// if ORB-SLAM initialization is successful
                {
                    cv::Mat im(480,640,CV_8UC3,gARTImage);	// Construct mat type image
                    
                    if(im.empty())
                    {
                    cerr << endl << "Failed to load image from ARToolkit."<<endl;
                    return;
                    }
                    
                    
                    // Get the current timestamp
                    std::chrono::system_clock::time_point t_epoch;
                    std::chrono::duration<double,std::ratio<1,1000000>> duration_micr_sec = std::chrono::system_clock::now() - t_epoch;
                    double tframe = duration_micr_sec.count() / 1000000.0;	
                    
                    cv::Mat Tcm((*ptr_SLAM).TrackMonocular(im,tframe,0)); // Pass the picture and timestamp to the SLAM system to get the transformation matrix of the current template coordinate system to the camera coordinate system
                    
                    if((*ptr_State)==ORB_SLAM2::System::OK)	// Position is valid (SLAM tracking is not lost)
                    {
                        cout<<"Tracking without losing Tcm:"<<endl<<Tcm<<endl;
                        for(int m=0;m<3;m++){
                            for(int n=0;n<4;n++){
                                markersNFT[i].trans[m][n]=(ARdouble)Tcm.at<float>(m,n);
                            }
                        }
                        
                        markersNFT[i].valid = TRUE;	// Change the tracking status to successful
                        
                        // Filter the pose estimate.
                        if (markersNFT[i].ftmi) {		// Perform filtering optimization for pose results
                            if (arFilterTransMat(markersNFT[i].ftmi, markersNFT[i].trans, 0) < 0) {
                                        ARLOGe("arFilterTransMat error with marker %d.\n", i);
                            }
                        }		      

                        // We have a new pose, so set that.
                        arglCameraViewRH((const ARdouble (*)[4])markersNFT[i].trans, markersNFT[i].pose.T, VIEW_SCALEFACTOR);
                    }
                }
		
                if (markersNFT[i].validPrev) {	// last success?
                    // Marker has ceased to be visible, tell any dependent objects.	// marker from recognition to loss
                    // --->
                }
            }  
            
            
        }
		// Tell GLUT the display has changed.
		glutPostRedisplay();			// tell display () to display
	}
}

//
//	This function is called on events when the visibility of the
//	GLUT window changes (including when it first becomes visible).
//
static void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) {
		glutIdleFunc(mainLoop);		// After testing, the mainloop will always be executed
	} else {
		glutIdleFunc(NULL);
	}
}

//
//	This function is called when the
//	GLUT window is resized.
//
static void Reshape(int w, int h)
{
    gWindowW = w;
    gWindowH = h;
    
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	
	// Call through to anyone else who needs to know about window sizing here.
}

//
// This function is called when the window needs redrawing.
//
static void Display(void)
{
    //static int test = 0;
    int i;
    
	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.
    
    arglPixelBufferDataUpload(gArglSettings, gARTImage);	// New frame captured this time
	arglDispImage(gArglSettings);
	
	gARTImage = NULL; // Invalidate image data.		// Invalid image pointer after setting
				
    // Set up 3D mode.
	glMatrixMode(GL_PROJECTION);
#ifdef ARDOUBLE_IS_FLOAT
	glLoadMatrixf(cameraLens);
#else
	glLoadMatrixd(cameraLens);
#endif
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glEnable(GL_DEPTH_TEST);

    // Set any initial per-frame GL state you require here.
    // --->
    
    // Lighting and geometry that moves with the camera should be added here.
    // (I.e. should be specified before marker pose transform.)
    // --->
    
    for (i = 0; i < markersNFTCount; i++) {
        
        if (markersNFT[i].valid) {			// Displayed only when the current marker tracking is valid
        
#ifdef ARDOUBLE_IS_FLOAT
            glLoadMatrixf(markersNFT[i].pose.T);
#else
            glLoadMatrixd(markersNFT[i].pose.T);	// display finally uses markersNFT [i] .pose.T
#endif
	    
            // All lighting and geometry to be drawn relative to the marker goes here.
            // --->
            DrawCube();
        }
    }
    
    // Set up 2D mode.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, (GLdouble)gWindowW, 0, (GLdouble)gWindowH, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    // Add your own 2D overlays here.
    // --->
    
	glutSwapBuffers();
}
