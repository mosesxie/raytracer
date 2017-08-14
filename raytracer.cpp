#include "raytracer.hpp"

using namespace raytrace;

float mix(const float &a, const float &b, const float &mix)
{
	return b * mix + a * (1 - mix);
}

Vect3F refractRay(Vect3F const &W, Vect3F const &pt, Vect3F const &N,
		float const &eta1, float const &eta2, Shape const &shape){
	float etar = eta1 / eta2;
	float a = etar * -1;
	float wn = W.dotProduct(N);
	float radsq = 1 - (etar * etar) * (1 - wn * wn);
	Vect3F T(0.0);
	if(radsq > 0){
		float b = (etar * wn) - sqrt(radsq);
		T = (W * a) + (N * b);
	}
	return T;
}

Ray refractExit(Vect3F const &W, Vect3F const &pt, float const &eta_in, Shape const &shape){
	Ray refR;
	Vect3F T1 = refractRay(W, pt, (pt - shape.getPos()).normalize(), 1.0, eta_in, shape);
	float t1_sum = T1.sum();
	if(t1_sum != 0.0){
		Vect3F exit = pt + T1 * 2 * (shape.getPos() - pt).dotProduct(T1);
		Vect3F Nin = shape.getPos() - exit;
		Vect3F T2 = refractRay(T1 * -1, exit, Nin, eta_in, 1.0, shape);
		refR = Ray(exit, T2);
	}
	return refR;
}

bool Sphere::intersect(const Ray &ray, float &t0, float &t1)
{
	Vect3F L = ray.getOrigin(), D = ray.getDirection();
	Vect3F tv = getPos() - L;
	float v = tv.dotProduct(D);
	if(v < 0)
		return false;
	float dsq = tv.dotProduct(tv) - (v*v);
	if(dsq > radSqrd)
		return false;
	float b = sqrt(radSqrd - dsq);
	t0 = v - b;
	t1 = v + b;
	return true;
}
bool Triangle::intersect(const Ray &ray, float &t0, float &t1)
{
	Vect3F E1 = B - A;
	Vect3F E2 = C - A;
	Vect3F dir = ray.getDirection();
	Vect3F ori = ray.getOrigin();
	Vect3F pVect = dir.crossProduct(E2);
	float det = E1.dotProduct(pVect);
	float invDet = 1/det;
	if(det < 0.0000001f) return false;
	Vect3F tVect = ori - A;
	float u = tVect.dotProduct(pVect)*invDet;
	if(u < 0.0 || u > 1) return false;
	Vect3F qVect = tVect.crossProduct(E1);
	float v = dir.dotProduct(qVect)*invDet;
	if(v < 0 || u + v > 1) return false;
	t0 = E2.dotProduct(qVect)*invDet;
	return true;
}

Vect3F Scene::draw(const Ray &ray, const int &level)
{
	Vect3F ori = ray.getOrigin();
	Vect3F dir = ray.getDirection();
	float tNear = INFINITY;
	Vect3F pixelValue;
	const Shape *shape = NULL;
	if(!shapes.empty())
	{
		for(unsigned i=0; i < shapes.size(); i++)
		{
			float t0 = INFINITY;
			float t1 = INFINITY;
			if(shapes[i]->intersect(ray, t0, t1))
			{
				if(t0 < 0)
				{
					t0 = t1;
				}
				if(t0 < tNear)
				{
					tNear = t0;
					shape = shapes[i];
				}
			}	
		}
	}
	if(!shape)
		return Vect3F(0.0);
	Vect3F ptos = ori + (dir * tNear);
	Vect3F snorm = (ptos - shape->getPos()).normalize();
	float bias = 1.0e-4;
	for(unsigned i=0; i < shape->getLights().size(); i++)
	{
		Vect3F ptl = (shape->getLights()[i].getPosition()-ptos).normalize();
		Vect3F eml = shape->getLights()[i].getColor();
		if(snorm.dotProduct(ptl) > 0)
		{
			pixelValue += eml * shape->getMat().getKD() * std::max(float(0), snorm.dotProduct(ptl));
			pixelValue += eml * shape->getMat().getKS()
				* std::max(float(0), (float)pow(snorm.dotProduct(ptl), shape->getMat().getLevel()));
		}
	}
	if(level > 0)
	{
		float faceLevel = dir.dotProduct(snorm);
		float lenseEffect = mix(pow(1 - faceLevel, 4), 1, 0.1);
		Vect3F reflectDirect(dir - snorm * 2 * faceLevel);
		reflectDirect.normalize();
		Ray reflectRay((ptos + snorm * bias), reflectDirect);
		Vect3F reflection(draw(reflectRay, level - 1));
		Ray refractRay = refractExit(dir * -1, ptos, shape->getMat().getETA(), *shape);
		Vect3F refraction(0.0);
		if(refractRay.getDirection().sum() && shape->getMat().getKO().sum()){
			refraction = draw(refractRay, level - 1);
		}
		pixelValue += (reflection * lenseEffect * shape->getMat().getKR()
				+ refraction * lenseEffect * shape->getMat().getKO()) * shape->getMat().getKD();
	}
	return pixelValue;
}

std::vector<std::string> Scene::split(const std::string &str, const char &delimit)
{
	std::vector<std::string> result;
	std::string data;
	std::stringstream ss(str);
	while(getline(ss, data, delimit))
	{
		result.push_back(data);
	}
	return result;
}

Scene::Scene(const std::string &fileName, const int &h, const int &w)
{
	height = h; width = w;
	int lineCount = 1;
	std::vector<std::string> lineSplit;
	std::ifstream infile;
	infile.open(fileName, std::ios::binary | std::ios::in);
	if(!infile)
	{
		std::cerr << "Error: file " << fileName << " does not exist." << std::endl;
		exit(102);
	}
	//Reading from file
	//Read from camera
	float x,y,z;
	std::string line;
	getline(infile, line);
	lineSplit = split(line, ' ');
	if(lineSplit[0] != "eye")
	{
		std::cerr << "Error: incorrect file format in line: " << lineCount << std::endl;
		std::cerr << "Line shows: " << line << std::endl;
		exit(103);
	}
	x = std::stof(lineSplit[1]);
	y = std::stof(lineSplit[2]);
	z = std::stof(lineSplit[3]);
	lineCount++;
	Vect3F eye(x,y,z);
	
	getline(infile, line);
	lineSplit = split(line, ' ');
	if(lineSplit[0] != "look")
	{
		std::cerr << "Error: incorrect file format in line: " << lineCount << std::endl;
		std::cerr << "Line shows: " << line << std::endl;
		exit(103);
	}
	x = std::stof(lineSplit[1]);
	y = std::stof(lineSplit[2]);
	z = std::stof(lineSplit[3]);
	lineCount++;
	Vect3F lookAt(x,y,z);
	
	getline(infile, line);
	lineSplit = split(line, ' ');
	if(lineSplit[0] != "up")
	{
		std::cerr << "Error: incorrect file format in line: " << lineCount << std::endl;
		std::cerr << "Line shows: " << line << std::endl;
		exit(103);
	}
	x = std::stof(lineSplit[1]);
	y = std::stof(lineSplit[2]);
	z = std::stof(lineSplit[3]);
	lineCount++;
	Vect3F up(x,y,z);
	
	getline(infile, line);
	lineSplit = split(line, ' ');
	if(lineSplit[0] != "fl")
	{
		std::cerr << "Error: incorrect file format in line: " << lineCount << std::endl;
		std::cerr << "Line shows: " << line << std::endl;
		exit(103);
	}
	float focalLength = std::stof(lineSplit[1]);
	lineCount++;
	
	Camera c(eye, lookAt, up, focalLength);
	camera = c;
	
	getline(infile, line);
	lineSplit = split(line, ' ');
	if(lineSplit[0] != "plane")
	{
		std::cerr << "Error: incorrect file format in line: " << lineCount << std::endl;
		std::cerr << "Line shows: " << line << std::endl;
		exit(103);
	}
	lineCount++;
	camera.setRight(std::stof(lineSplit[1]));
	camera.setLeft(std::stof(lineSplit[2]));
	camera.setTop(std::stof(lineSplit[3]));
	camera.setBottom(std::stof(lineSplit[4]));
	std::cout << "Reading light objects..." << std::endl;
	//Reading light objects
	getline(infile, line);
	lineSplit = split(line, ' ');
	std::vector<Light> lights;
	while(lineSplit[0] == "light")
	{
		Light light(Vect3F(std::stof(lineSplit[1]),
						   std::stof(lineSplit[2]),
						   std::stof(lineSplit[3])),
					Vect3F(std::stof(lineSplit[4]),
						   std::stof(lineSplit[5]),
						   std::stof(lineSplit[6])));
		lights.push_back(light);
		lineCount++;
		getline(infile, line);
		lineSplit = split(line, ' ');
	}
	std::cout << "Reading material objects..." << std::endl;
	while(lineSplit[0] == "material")
	{
		Material material(Vect3F(std::stof(lineSplit[1]),
								 std::stof(lineSplit[2]),
								 std::stof(lineSplit[3])),
						  Vect3F(std::stof(lineSplit[4]),
								 std::stof(lineSplit[5]),
								 std::stof(lineSplit[6])),
						  Vect3F(std::stof(lineSplit[7]),
								 std::stof(lineSplit[8]),
								 std::stof(lineSplit[9])),
						  Vect3F(std::stof(lineSplit[10]),
								 std::stof(lineSplit[11]),
								 std::stof(lineSplit[12])),
						  Vect3F(std::stof(lineSplit[13]),
								 std::stof(lineSplit[14]),
								 std::stof(lineSplit[15])),
						  std::stof(lineSplit[16]),
						  std::stof(lineSplit[17]));
		lineCount++;
		getline(infile, line);
		lineSplit = split(line, ' ');
		std::cout << "Reading shape objects..." << std::endl;
		while(lineSplit[0] == "sphere" || lineSplit[0] == "triangle")
		{
			if(lineSplit[0] == "sphere")
			{
				Sphere * sphere = new Sphere(Vect3F(std::stof(lineSplit[1]),
						  std::stof(lineSplit[2]), std::stof(lineSplit[3])),
						  std::stof(lineSplit[4]), material, lights);
				shapes.push_back(sphere);
			}
			if(lineSplit[0] == "triangle")
			{
				Triangle * triangle = new Triangle(
				Vect3F(std::stof(lineSplit[1]),
				       std::stof(lineSplit[2]),
					   std::stof(lineSplit[3])),
				Vect3F(std::stof(lineSplit[4]),
				       std::stof(lineSplit[5]),
					   std::stof(lineSplit[6])),
				Vect3F(std::stof(lineSplit[7]),
					   std::stof(lineSplit[8]),
					   std::stof(lineSplit[9])),
				Vect3F(std::stof(lineSplit[10]),
					   std::stof(lineSplit[11]),
					   std::stof(lineSplit[12])),
				material, lights);
				shapes.push_back(triangle);
			}
			lineCount++;
			getline(infile, line);
			lineSplit = split(line, ' ');
		}
	}
	std::cout << "Finished Reading file..." << std::endl;
	std::cout << "Number of shapes: " << shapes.size() << std::endl;
	infile.close();
}

void Scene::render(void)
{
	Vect3F W = camera.getW();
	Vect3F U = camera.getU();
	Vect3F V = camera.getV();
	Vect3F E = camera.getEye();
	float fl = camera.getFocalLength();
	float r = camera.getRight(),
		  l = camera.getLeft(),
		  t = camera.getTop(),
		  b = camera.getBottom();
	float px = 0.0, py = 0.0;
	Vect3F ori, dir;
	Vect3F *image = new Vect3F[width * height], *pixel = image;
	float aspectRatio = width/(float)height;
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++, pixel++)
		{
			px = (float)j/(width-1)*(r-l)+l;
			py = (float)i/(height-1)*(t-b)+b;
			px *= aspectRatio;
			ori = E + (W * -fl) + (U * px) + (V * py);
			dir = (ori - E).normalize();
			Ray ray(ori, dir);
			*pixel = draw(ray, 5);			
		}
		std::cout << "Percent Completed: " << (i/(float)height) * 100 << "\r";
	}
	std::ofstream outFile;
	outFile.open("output.ppm", std::ios::binary | std::ios::out);
	outFile << "P6\n" << width << " " << height << "\n255\n";
	for (int i = 0; i < width * height; ++i) {
		outFile << (unsigned char)(std::min(float(1), image[i].getX()) * 255) <<
				   (unsigned char)(std::min(float(1), image[i].getY()) * 255) <<
				   (unsigned char)(std::min(float(1), image[i].getZ()) * 255);
	}
	outFile.close();
	delete [] image;
}


int main(int agrc, char**agrs)
{
	if(agrc != 4)
	{
		std::cerr << "Usage: \'raytracer [filename.txt] [width] [height]\'" << std::endl;
		return 1;
	}
	Scene scene(agrs[1], std::stoi(agrs[3]), std::stoi(agrs[2]));
	scene.render();
}












