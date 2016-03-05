#ifndef MYCALIB3D_HPP
#define MYCALIB3D_HPP
namespace cv {
namespace fisheye
{
    enum{
        CALIB_USE_INTRINSIC_GUESS   = 1,
        CALIB_RECOMPUTE_EXTRINSIC   = 2,
        CALIB_CHECK_COND            = 4,
        CALIB_FIX_SKEW              = 8,
        CALIB_FIX_K1                = 16,
        CALIB_FIX_K2                = 32,
        CALIB_FIX_K3                = 64,
        CALIB_FIX_K4                = 128,
        CALIB_FIX_INTRINSIC         = 256
    };

    //! projects 3D points using fisheye model
    //void projectPoints(InputArray objectPoints, OutputArray imagePoints, const Affine3d& affine,
    //    InputArray K, InputArray D, double alpha = 0, OutputArray jacobian = noArray());

    //! projects points using fisheye model
    void projectPoints(InputArray objectPoints, OutputArray imagePoints, InputArray rvec, InputArray tvec,
        InputArray K, InputArray D, double alpha = 0, OutputArray jacobian = noArray());

    //! distorts 2D points using fisheye model
     void distortPoints(InputArray undistorted, OutputArray distorted, InputArray K, InputArray D, double alpha = 0);

    //! undistorts 2D points using fisheye model
     void undistortPoints(InputArray distorted, OutputArray undistorted,
        InputArray K, InputArray D, InputArray R = noArray(), InputArray P  = noArray());

    //! computing undistortion and rectification maps for image transform by cv::remap()
    //! If D is empty zero distortion is used, if R or P is empty identity matrixes are used
     void initUndistortRectifyMap(InputArray K, InputArray D, InputArray R, InputArray P,
        const cv::Size& size, int m1type, OutputArray map1, OutputArray map2);

    //! undistorts image, optionally changes resolution and camera matrix. If Knew zero identity matrix is used
     void undistortImage(InputArray distorted, OutputArray undistorted,
        InputArray K, InputArray D, InputArray Knew = cv::noArray(), const Size& new_size = Size());

    //! estimates new camera matrix for undistortion or rectification
     void estimateNewCameraMatrixForUndistortRectify(InputArray K, InputArray D, const Size &image_size, InputArray R,
        OutputArray P, double balance = 0.0, const Size& new_size = Size(), double fov_scale = 1.0);

    //! performs camera calibaration
     double calibrate(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints, const Size& image_size,
        InputOutputArray K, InputOutputArray D, OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs, int flags = 0,
            TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

    //! stereo rectification estimation
     void stereoRectify(InputArray K1, InputArray D1, InputArray K2, InputArray D2, const Size &imageSize, InputArray R, InputArray tvec,
        OutputArray R1, OutputArray R2, OutputArray P1, OutputArray P2, OutputArray Q, int flags, const Size &newImageSize = Size(),
        double balance = 0.0, double fov_scale = 1.0);

    //! performs stereo calibaration
     double stereoCalibrate(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints1, InputArrayOfArrays imagePoints2,
                                  InputOutputArray K1, InputOutputArray D1, InputOutputArray K2, InputOutputArray D2, Size imageSize,
                                  OutputArray R, OutputArray T, int flags = CALIB_FIX_INTRINSIC,
                                  TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

}
} // cv
#endif