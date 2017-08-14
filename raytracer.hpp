#ifndef RAYTRACER_HPP
#define RAYTRACER_HPP
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
namespace raytrace
{
	template<typename T>
	class Vector
	{
		private:
			T x, y, z;
		public:
			Vector():
				x(0.0), y(0.0), z(0.0){}
			Vector(const T &n):
				x(n), y(n), z(n){}
			Vector(const T &s, const T &t, const T &u):
				x(s), y(t), z(u){}
			T getX() { return x; }
			T getY() { return y; }
			T getZ() { return z; }
			Vector<T> operator + (const T &n) const
			{
				return Vector<T>(x+n, y+n, z+n);
			}
			Vector<T> operator + (const Vector<T> &n) const
			{
				return Vector<T>(x+n.x, y+n.y, z+n.z);
			}
			Vector<T> operator - (const T &n) const
			{
				return Vector<T>(x-n, y-n, z-n);
			}
			Vector<T> operator - (const Vector<T> &n) const
			{
				return Vector<T>(x-n.x,y-n.y,z-n.z);
			}
			Vector<T> operator * (const T &n) const
			{
				return Vector<T>(x*n, y*n, z*n);
			}
			Vector<T> operator * (const Vector<T> &n) const
			{
				return Vector<T>(x*n.x, y*n.y, z*n.z);
			}
			Vector<T> operator / (const T &n) const
			{
				if(n < 0)
				{
					std::cerr << "Error: divide by zero." << std::endl;
					exit(101);
				}
				return Vector<T>(x/n,y/n,z/n);
			}
			Vector<T> operator / (const Vector<T> &n) const
			{
				if(n.x < 0 || n.y < 0 || n.z < 0)
				{
					std::cerr << "Error: divide by zero." << std::endl;
					exit(101);
				}
				return Vector<T>(x/n.x,y/n.y,z/n.z);
			}
			Vector<T> operator += (const T &n)
			{
				return Vector<T>(x += n, y += n,z += n);
			}
			Vector<T> operator += (const Vector<T> &n)
			{
				return Vector<T>(x += n.x, y += n.y, z += n.z);
			}
			friend std::ostream & operator << (std::ostream &os, const Vector<T> &v)
			{
				os << v.x << " " << v.y << " " << v.z;
				return os;
			}
			T dotProduct(const Vector<T> &v) const
			{
				return (x*v.x + y*v.y + z*v.z);
			}
			Vector<T> crossProduct(const Vector<T> &v) const
			{
				return Vector<T>((y*v.z) - (z*v.y),
								 (z*v.x) - (x*v.z),
								 (x*v.y) - (y*v.x));
			}
			T magnitude(void) const
			{
				return sqrt(x*x + y*y + z*z);
			}
			Vector<T> &normalize(void)
			{
				T magInv = 1/magnitude();
				x*=magInv, y*=magInv, z*=magInv;
				return *this;
			}
			T sum() const
			{
				return x+y+z;
			}
			
	};
	typedef Vector<float> Vect3F;
	class Camera
	{
		private:
		Vect3F eye, lookAt, up;
		float right, left, top, bottom, focalLength;
		public:
		Camera():
			right(0.0), left(0.0), top(0.0), bottom(0.0), focalLength(0.0){}
		Camera(const Vect3F &e,
			   const Vect3F &la,
			   const Vect3F &u,
			   const float &fl):
			   eye(e), lookAt(la), up(u), right(0.0), left(0.0),
			   top(0.0), bottom(0.0), focalLength(fl){}
		Vect3F getEye() const { return eye; }
		//Camera coordinates
		Vect3F getW() const { return (eye - lookAt).normalize(); }
		Vect3F getU() const { return (up.crossProduct(getW())).normalize(); }
		Vect3F getV() const { return getW().crossProduct(getU()); }
		//Accessor and mutator methods
		float getRight() const { return right; }
		float getLeft() const { return left; }
		float getTop() const { return top; }
		float getBottom() const { return bottom; }
		float getFocalLength() const { return focalLength; }
		void setRight(const float &r) { right = r; }
		void setLeft(const float &l) { left = l; }
		void setTop(const float &t) { top = t; }
		void setBottom(const float &b) { bottom = b; }
	};
	class Ray
	{
		private:
		Vect3F origin, direction;
		public:
		Ray(){}
		Ray(const Vect3F &ori, const Vect3F &dir):
			origin(ori), direction(dir){}
		Vect3F getOrigin() const { return origin; }
		Vect3F getDirection() const { return direction; }
	};
	class Light
	{
		private:
		Vect3F position, color;
		public:
		Light(){}
		Light(const Vect3F pos, const Vect3F col):
			position(pos), color(col){}
		Vect3F getPosition() const { return position; }
		Vect3F getColor() const { return color; }
	};
	class Material
	{
		private:
		Vect3F KA, KD, KS, KR, KO;
		float level, eta;
		public:
		Material(): level(0.0) {}
		Material(const Vect3F &ka, 
				 const Vect3F &kd,
				 const Vect3F &ks,
				 const Vect3F &kr,
				 const Vect3F &ko,
				 const float &lv,
				 const float &e):
		KA(ka), KD(kd), KS(ks), KR(kr), KO(ko), level(lv), eta(e){}
		Vect3F getKA() const { return KA; }
		Vect3F getKD() const { return KD; }
		Vect3F getKS() const { return KS; }
		Vect3F getKR() const { return KR; }
		Vect3F getKO() const { return KO; }
		float getLevel() const { return level; }
		float getETA() const { return eta; }
	};	
	//Object base class
	class Shape
	{
		protected:
		Vect3F position;
		Material material;
		std::vector<Light> lights;
		public:
		Shape(){}
		virtual ~Shape(){}
		virtual bool intersect(const Ray &ray,
			float &t0, float &t1) = 0;
		Vect3F getPos() const { return position; }
		Material getMat() const { return material; }
		std::vector<Light> getLights() const { return lights; }
	};
	class Sphere : public Shape
	{
		private:
		float radius, radSqrd;
		public:
		Sphere():
			radius(0.0), radSqrd(0.0){}
		Sphere(const Vect3F &pos,
			   const float &r,
			   const Material &mat,
			   const std::vector<Light> &lights):
			   radius(r), radSqrd(r*r)
			{
				Shape::position = pos;
				Shape::material = mat;
				Shape::lights = lights;
			}
		bool intersect(const Ray &ray, float &t0, float &t1);
	};
	class Triangle : public Shape
	{
		private:
		Vect3F A, B, C, surfaceNorm;
		public:
		Triangle(){}
		Triangle(const Vect3F &a,
			     const Vect3F &b,
			     const Vect3F &c,
				 const Vect3F &sn,
			     const Material &mat,
			     const std::vector<Light> &lights):
			     A(a), B(b), C(c), surfaceNorm(sn)
			{
				Shape::material = mat;
				Shape::lights = lights;
			}
		bool intersect(const Ray &ray, float &t0, float &t1);
		Vect3F getSideA() const { return A; }
		Vect3F getSideB() const { return B; }
		Vect3F getSideC() const { return C; }
	};
	class Scene
	{
		private:
		std::vector<Shape*> shapes;
		Camera camera;
		int height, width;
		std::vector<std::string> split(const std::string &str, const char &delimit);
		public:
		Scene(const std::string &fileName, const int &h, const int &w);
		void render(void);
		Vect3F draw(const Ray &ray, const int &level);
	};
}
#endif