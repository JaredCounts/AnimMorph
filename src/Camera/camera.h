// Arcball camera by Eugene Hsu
// Based on 6.839 sample code for rotation code.
// Extended to handle translation (MIDDLE) and scale (RIGHT)

#pragma once

#include "../linearAlgebra.h"

#include "../mouseable.h"

class Camera: public Mouseable
{
public:

    Camera();

    // You must call all of the Set*() functions before you use this!
    // I didn't put it into the constructor because it's inconvenient
    // to initialize stuff in my opengl application.
    
    void SetDimensions(int w, int h);
    void SetViewport(int x, int y, int w, int h);
    void SetPerspective(float fovy);

    // Call from whatever UI toolkit
    virtual void MouseClick(MouseButton button, int x, int y);
    virtual void MouseDrag(MouseButton button, int prevX, int prevY, int x, int y);
    virtual void MouseRelease(int x, int y);
	virtual void MouseScroll(double xOffset, double yOffset);

	Matrix4f projectionMatrix() const;
	Matrix4f viewMatrix() const;

    // Set for relevant vars
    void SetCenter(const Vector3f& center);
    void SetRotation(const Matrix4f& rotation);
    void SetDistance(const float distance);

    // Get for relevant vars
    Vector3f GetCenter() const { return mCurrentCenter; }
    Matrix4f GetRotation() const { return mCurrentRot; }
    float GetDistance() const { return mCurrentDistance; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
private:

    // States 
    int     mDimensions[2];
    int     mStartClick[2];

    // For rotation
    Matrix4f mStartRot;
    Matrix4f mCurrentRot;

    // For translation
    float   mPerspective[2];
    int     mViewport[4];
    Vector3f mStartCenter;
    Vector3f mCurrentCenter;

    // For zoom
    float   mStartDistance;
    float   mCurrentDistance;

	const Vector3f UP = Vector3f(0, 1, 0);

    void ArcBallRotation(int x, int y);
    void PlaneTranslation(int x, int y);
    void DistanceZoom(int x, int y);
};
typedef std::shared_ptr<Camera> P_Camera;