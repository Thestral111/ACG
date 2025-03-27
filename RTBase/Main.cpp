

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>

void runTests()
{
	// Add test code here
	// Create an AABB with min and max points.
	AABB box;
	box.min = Vec3(-1.0f, -1.0f, -1.0f);
	box.max = Vec3(1.0f, 1.0f, 1.0f);

	// Test ray that should hit the box.
	// Starting at (0,0,-5) and pointing in the positive z direction.
	Ray ray1(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
	float t;
	bool hit1 = box.rayAABB(ray1, t);
	std::cout << "Ray 1 hits the box: " << (hit1 ? "Yes" : "No") << std::endl;
	if (hit1) {
		std::cout << "Entry distance t: " << t << std::endl;
	}

	// Test ray that should miss the box.
	// Starting at (5,5,-5) and pointing in the positive z direction.
	Ray ray2(Vec3(5.0f, 5.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
	bool hit2 = box.rayAABB(ray2);
	std::cout << "Ray 2 hits the box: " << (hit2 ? "Yes" : "No") << std::endl;

	// Output the computed area of the box.
	std::cout << "Surface area of the AABB: " << box.area() << std::endl;
}

int main(int argc, char *argv[])
{
	// Add call to tests if required
	runTests();
	
	// Initialize default parameters
	//std::string sceneName = "cornell-box";
	//std::string sceneName = "living-room";
	//std::string sceneName = "kitchen";
	std::string sceneName = "MaterialsScene";
	std::string filename = "GI.hdr";
	unsigned int SPP = 8192; //8192

	if (argc > 1)
	{
		std::unordered_map<std::string, std::string> args;
		for (int i = 1; i < argc; ++i)
		{
			std::string arg = argv[i];
			if (!arg.empty() && arg[0] == '-')
			{
				std::string argName = arg;
				if (i + 1 < argc)
				{
					std::string argValue = argv[++i];
					args[argName] = argValue;
				} else
				{
					std::cerr << "Error: Missing value for argument '" << arg << "'\n";
				}
			} else
			{
				std::cerr << "Warning: Ignoring unexpected argument '" << arg << "'\n";
			}
		}
		for (const auto& pair : args)
		{
			if (pair.first == "-scene")
			{
				sceneName = pair.second;
			}
			if (pair.first == "-outputFilename")
			{
				filename = pair.second;
			}
			if (pair.first == "-SPP")
			{
				SPP = stoi(pair.second);
			}
		}
	}
	Scene* scene = loadScene(sceneName);
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", 1.0f);
	RayTracer rt;
	rt.init(scene, &canvas);
	bool running = true;
	GamesEngineeringBase::Timer timer;
	while (running)
	{
		canvas.checkInput();
		canvas.clear();
		if (canvas.keyPressed(VK_ESCAPE))
		{
			break;
		}
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward();
			rt.clear();
		}
		if (canvas.keyPressed('S'))
		{
			viewcamera.back();
			rt.clear();
		}
		if (canvas.keyPressed('A'))
		{
			viewcamera.left();
			rt.clear();
		}
		if (canvas.keyPressed('D'))
		{
			viewcamera.right();
			rt.clear();
		}
		if (canvas.keyPressed('E'))
		{
			viewcamera.flyUp();
			rt.clear();
		}
		if (canvas.keyPressed('Q'))
		{
			viewcamera.flyDown();
			rt.clear();
		}
		// Time how long a render call takes
		timer.reset();
		rt.render();
		float t = timer.dt();
		// Write
		std::cout << t << std::endl;
		if (canvas.keyPressed('P'))
		{
			rt.saveHDR(filename);
		}
		if (canvas.keyPressed('L'))
		{
			size_t pos = filename.find_last_of('.');
			std::string ldrFilename = filename.substr(0, pos) + ".png";
			rt.savePNG(ldrFilename);
		}
		if (SPP == rt.getSPP())
		{
			rt.saveHDR(filename);
			break;
		}
		canvas.present();
	}
	return 0;
}