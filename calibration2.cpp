int calibrar(char* b,char* e,int nI,int xc,int yc )
{
int xCorners = 0;
int yCorners = 0;
int iImg;
int width;
int height;
int numImages=nI;

sprintf(base, “%s”, b);
sprintf(end, “%s”, e);
numImages = nI;
xCorners = xc;
yCorners = yc;

printf(“Calibrando …\n”);

cornersArray = new CvPoint2D32f[xCorners*yCorners*numImages];
corners3d = new CvPoint3D32f[xCorners*yCorners*numImages];
int *cornersFound = new int[numImages];
distortion = new float[4];
camera_matrix = new float[9];
translation_vectors = new float[3*numImages];
rotation_matrices = new float[9*numImages];
InitCorners3d(corners3d, xCorners, yCorners, numImages);
cvNamedWindow(“Image”, 0);
cvNamedWindow(“Undistorted”, 0);
CvPoint2D32f *nextCornersArray = cornersArray;

for (iImg = 0; iImg < numImages; iImg++)
{
// Load all the new images into there data structures
sprintf(filename, “%s%d%s”, base, iImg, end);
printf(“Procesando imagen imagen: %s\n”,filename);
inicio = cvLoadImage(filename);
width = inicio->width;
height = inicio->height;
if (inicio->depth != IPL_DEPTH_8U)
{
fprintf(stderr, “image, %s, depth not IPL_DEPTH_8U\n”,filename);
exit(1);
}

bw = cvCreateImage(cvSize(inicio->width, inicio->height),IPL_DEPTH_8U, 1);
cvConvertImage(inicio, bw);
threshold = cvCreateImage(cvSize(inicio->width,inicio->height), IPL_DEPTH_8U, 1);
// Feedback of images loaded for sanity
cvShowImage(“Image”, inicio);
// Start calibration of points
cornersFound[iImg] = xCorners * yCorners;
// nextCornersArray is a pointer into cornersArray to the lastslot open
// this will overflow if cvchessboard ever gives us morecorners then expected
int bResult = cvFindChessBoardCornerGuesses(bw, threshold,NULL, cvSize(xCorners, yCorners), nextCornersArray, &cornersFound[iImg]);
cvFindCornerSubPix(bw, nextCornersArray, cornersFound[iImg],cvSize(5,5), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
// more visual feedback to confirm correctness
DrawCircles(inicio, nextCornersArray, cornersFound[iImg],“Image”, -1);
cvShowImage(“Image”, inicio);
sprintf(filename, “%s%d%s%s”, base, iImg,“_findChessBoardCornerGuesses”, end);
cvSaveImage(filename, inicio);
if (!bResult)
{
fprintf(stderr, “Did not find expected number of points… bailing\n”);
cvWaitKey(0);
}
printf(“Fin imagen: %s\n”,filename);
cvWaitKey(0);

// update so that nextCornersArray points to the next open slotin cornersArray
nextCornersArray += cornersFound[iImg];
cvReleaseImage(&inicio);
cvReleaseImage(&bw);
cvReleaseImage(&threshold);
}
cvCalibrateCamera(numImages, cornersFound, cvSize(width, height),
cornersArray, corners3d, distortion, camera_matrix,
translation_vectors,
rotation_matrices, 0);
PrintIntrinsics(distortion, camera_matrix);
CvPoint2D32f *backprojPts2d = ConvertWorldToPixel(corners3d, numImages,
cornersFound, camera_matrix, translation_vectors, rotation_matrices);
for (iImg = 0; iImg < numImages; iImg++)
{
// Load all the new images into there data structures
sprintf(filename, “%s%d%s”, base, iImg, end);
inicio = cvLoadImage(filename);
undistort = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U,3);
cvUnDistortOnce(inicio, undistort, camera_matrix, distortion, 1);
DrawCircles(inicio, &backprojPts2d[iImg*cornersFound[iImg]],cornersFound[iImg], “Image”, 1);
DrawCircles(undistort, &backprojPts2d[iImg*cornersFound[iImg]], cornersFound[iImg], “Undistorted”, 1);
sprintf(filename, “%s%d%s%s”, base, iImg, “_backproject”, end);
cvSaveImage(filename, inicio);
sprintf(filename, “%s%d%s%s”, base, iImg,“_backproject_undistorted”, end);
cvSaveImage(filename, undistort);
cvWaitKey(0);
cvReleaseImage(&inicio);
cvReleaseImage(&undistort);
}
cvWaitKey(0);
cvDestroyWindow(“Image”);

delete cornersArray;
delete corners3d;
delete cornersFound;
delete distortion;
delete camera_matrix;
delete translation_vectors;
delete rotation_matrices;

printf(“Fin Calibración”);
return 0;
}
