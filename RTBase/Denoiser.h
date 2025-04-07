#pragma once
#include <OpenImageDenoise/oidn.hpp>
#include "Imaging.h"
#include <cstring>
#include <iostream>

class Denoiser {
public:
    // Applies Intel Open Image Denoise to the film's color buffer.
    // It assumes that film->colorBuffer, film->albedoBuffer, and film->normalBuffer are valid.
    static void apply(Film* film) {
        int width = film->width;
        int height = film->height;

        // Create a new OIDN device.
        oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
        device.commit();
        
        // Create a ray tracing (RT) denoising filter.
        oidn::FilterRef filter = device.newFilter("RT");

        // Set the input noisy image.
        filter.setImage("color", film->colorBuffer, oidn::Format::Float3, width, height);

        // Set auxiliary buffers: albedo and normal.
        // These help preserve details and avoid blurring edges.
        filter.setImage("albedo", film->albedoBuffer, oidn::Format::Float3, width, height);
        filter.setImage("normal", film->normalBuffer, oidn::Format::Float3, width, height);

        // Allocate an output buffer.
        float* outputBuffer = new float[width * height * 3];
        filter.setImage("output", outputBuffer, oidn::Format::Float3, width, height);

        // Tell the filter that the input image is HDR.
        filter.set("hdr", true);

        // Commit filter settings.
        filter.commit();

        // Execute the denoising.
        filter.execute();

        // Check for errors.
        //const char* errorMessage = oidn::getError(device);
        // Check for errors.
        const char* errorMessage;
        if (device.getError(errorMessage) != oidn::Error::None) {
            std::cerr << "OIDN error: " << errorMessage << std::endl;
        }
        /*if (errorMessage) {
            std::cerr << "OIDN error: " << errorMessage << std::endl;
        }*/

        // Copy the denoised result back into the film's color buffer.
        std::memcpy(film->colorBuffer, outputBuffer, width * height * 3 * sizeof(float));
		cout << "Denoised image saved to colorBuffer" << endl;
        // Clean up the temporary output buffer.
        delete[] outputBuffer;
    }
};

