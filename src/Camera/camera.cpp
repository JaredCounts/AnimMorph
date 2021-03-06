#include "camera.h"

#include <iostream>

using namespace std;

Camera::Camera()
{
	mStartRot = Matrix4f::Identity();
	mCurrentRot = Matrix4f::Identity();

	zNear = 0.01f;
	zFar = 10000.f;
}

void Camera::SetDimensions(int w, int h)
{
	mDimensions[0] = w;
	mDimensions[1] = h;
}

void Camera::SetPerspective(float fovy)
{
	mPerspective[0] = fovy;
}

void Camera::SetViewport(int x, int y, int w, int h)
{
	mViewport[0] = x;
	mViewport[1] = y;
	mViewport[2] = w;
	mViewport[3] = h;
	mPerspective[1] = float(w) / h;
}

void Camera::SetCenter(const Vector3f& center)
{
	mStartCenter = mCurrentCenter = center;
}

void Camera::SetRotation(const Matrix4f& rotation)
{
	mStartRot = mCurrentRot = rotation;
}

void Camera::SetDistance(const float distance)
{
	mStartDistance = mCurrentDistance = distance;
}

Vector3f Camera::GetPointWorld(Vector2f pointView)
{
	float width = mViewport[2];
	float height = mViewport[3];

	// ray start/end positions on screen in camera space
	Vector4f screen1(
		2.0 * pointView.x() / width - 1.0, 
		1.0 - 2.0 * pointView.y() / height, 
		-1.0, 
		1);
	Vector4f screen2(
		screen1.x(),
		screen1.y(),
		1.0,
		1);

	Matrix4f projectionViewInverse = (projectionMatrix() * viewMatrix()).inverse();

	// convert ray start/end positions to homogeneous world space
	Vector4f world1 = projectionViewInverse * screen1;
	Vector4f world2 = projectionViewInverse * screen2;

	// get world space positions
	Vector3f world1H = world1.head(3) * (1.0 / world1.w());
	Vector3f world2H = world2.head(3) * (1.0 / world2.w());
	
	// direction
	Vector3f dir = world2H - world1H;

	// compute value along ray for X-Y plane
	float t = -world1H.z() / dir.z();

	// compute ray position on X-Y plane
	return world1H + dir * t;
}

void Camera::MouseClick(MouseButton button, int x, int y)
{
	mStartClick[0] = x;
	mStartClick[1] = y;

	switch (button)
	{
	case LEFT:
		mCurrentRot = mStartRot;
		break;
	case MIDDLE:
		mCurrentCenter = mStartCenter;
		break;
	case RIGHT:
		mCurrentDistance = mStartDistance;
		break;
	default:
		break;
	}
}

void Camera::MouseDrag(MouseButton button, int prevX, int prevY, int x, int y)
{
	switch (button)
	{
	case LEFT:
		PlaneTranslation(x, y);
		// xxx: should add a "is 2D?" toggle.
		//ArcBallRotation(x, y); // no rotations
		break;
	case MIDDLE:
		PlaneTranslation(x, y);
		break;
	case RIGHT:
		DistanceZoom(x, y);
		break;
	default:
		break;
	}
}


void Camera::MouseRelease(int x, int y)
{
	mStartRot = mCurrentRot;
	mStartCenter = mCurrentCenter;
	mStartDistance = mCurrentDistance;
}

void Camera::MouseScroll(double xOffset, double yOffset)
{ // XXX this can probably be combined with DistanceZoom
	const float MIN_DISTANCE = 0.1f;
	const float SCROLL_FACTOR = 0.05f;

	mCurrentDistance = mCurrentDistance + SCROLL_FACTOR * yOffset * mCurrentDistance;

	mCurrentDistance = max(MIN_DISTANCE, mCurrentDistance);
	mStartDistance = mCurrentDistance;
}


void Camera::ArcBallRotation(int x, int y)
{
 	// cout << x << " " << y << "\n";
	float sx, sy, sz, ex, ey, ez;
	float scale;
	float sl, el;
	float dotprod;

	// find vectors from center of window
	sx = mStartClick[0] - (mDimensions[0] / 2.f);
	sy = mStartClick[1] - (mDimensions[1] / 2.f);
	ex = x - (mDimensions[0] / 2.f);
	ey = y - (mDimensions[1] / 2.f);

	// invert y coordinates (raster versus device coordinates)
	sy = -sy;
	ey = -ey;

	// scale by inverse of size of window and magical sqrt2 factor
	if (mDimensions[0] > mDimensions[1]) {
		scale = (float)mDimensions[1];
	}
	else {
		scale = (float)mDimensions[0];
	}

	scale = 1.f / scale;

	sx *= scale;
	sy *= scale;
	ex *= scale;
	ey *= scale;

	// project points to unit circle
	sl = hypot(sx, sy);
	el = hypot(ex, ey);

	if (sl > 1.f) {
		sx /= sl;
		sy /= sl;
		sl = 1.0;
	}
	if (el > 1.f) {
		ex /= el;
		ey /= el;
		el = 1.f;
	}

	// project up to unit sphere - find Z coordinate
	sz = sqrt(1.0f - sl * sl);
	ez = sqrt(1.0f - el * el);

	// rotate (sx,sy,sz) into (ex,ey,ez)

	// compute angle from dot-product of unit vectors (and double it).
	// compute axis from cross product.
	dotprod = sx * ex + sy * ey + sz * ez;

	if (dotprod != 1)
	{
		Vector3f axis(sy * ez - ey * sz, sz * ex - ez * sx, sx * ey - ex * sy);
		axis.normalize();

		float angle = 2.0f * acos(dotprod);

		mCurrentRot = Affine3f(AngleAxisf(angle, axis)).matrix() * mStartRot;
	}
	else
	{
		mCurrentRot = mStartRot;
	}

}

void Camera::PlaneTranslation(int x, int y)
{
	// map window x,y into viewport x,y

	// start
	int sx = mStartClick[0] - mViewport[0];
	int sy = mStartClick[1] - mViewport[1];

	// current
	int cx = x - mViewport[0];
	int cy = y - mViewport[1];


	// compute "distance" of image plane (wrt projection matrix)
	float d = float(mViewport[3]) / 2.0f / tan(mPerspective[0] * M_PI / 180.0f / 2.0f);

	// compute up plane intersect of clickpoint (wrt fovy)
	float su = -sy + mViewport[3] / 2.0f;
	float cu = -cy + mViewport[3] / 2.0f;

	// compute right plane intersect of clickpoint (ASSUMED FOVY is 1)
	float sr = (sx - mViewport[2] / 2.0f);
	float cr = (cx - mViewport[2] / 2.0f);

	Vector2f move(cr - sr, cu - su);

	// this maps move
	move *= -mCurrentDistance / d;

	mCurrentCenter = mStartCenter +
		+ move[0] * Vector3f(mCurrentRot(0, 0), mCurrentRot(0, 1), mCurrentRot(0, 2))
		+ move[1] * Vector3f(mCurrentRot(1, 0), mCurrentRot(1, 1), mCurrentRot(1, 2));
}

Matrix4f Camera::projectionMatrix() const
{
	float fovYRadians = mPerspective[0] * M_PI / 180.f;
	float aspect = mPerspective[1];

	float yScale = 1.f / tanf(0.5f * fovYRadians);
	float xScale = yScale / aspect;

	Matrix4f perspective;
	perspective << 
		xScale, 0, 0, 0,
        0, yScale, 0, 0,
        0, 0, -(zFar+zNear)/(zFar-zNear), -2*zNear*zFar/(zFar-zNear),
        0, 0, -1, 0;

	return perspective;
}

Matrix4f Camera::viewMatrix() const
{
	// back up distance
	Vector3f eye(0,0,mCurrentDistance);
	Vector3f center = Vector3f::Zero();
	Vector3f up = UP;

	Vector3f z = (eye - center).normalized();
	Vector3f y = up;
	Vector3f x = y.cross(z);
	
	Matrix4f lookAt;
	lookAt << 
		x[0], x[1], x[2], -x.dot(eye),
		y[0], y[1], y[2], -y.dot(eye),
		z[0], z[1], z[2], -z.dot(eye),
		0,0,0,1;

	return lookAt * mCurrentRot * Affine3f(Translation3f(-mCurrentCenter)).matrix();
}

void Camera::DistanceZoom(int x, int y)
{
	int sy = mStartClick[1] - mViewport[1];
	int cy = y - mViewport[1];

	float delta = float(cy - sy) / mViewport[3];

	// exponential zoom factor
	mCurrentDistance = mStartDistance * exp(delta);
}
