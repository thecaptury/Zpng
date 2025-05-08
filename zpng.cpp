/** \file
    \brief Zpng - Experimental Lossless Image Compressor
    \copyright Copyright (c) 2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of Zpng nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

#define ZDICT_STATIC_LINKING_ONLY

#include "zpng.h"
#include "zstd/zdict.h"
#include "zstd/zstd.h"

#include <stdlib.h> // calloc
#include <string.h> // memset
#include <stdio.h>

typedef unsigned int uint;

//------------------------------------------------------------------------------
// Constants

// Higher compression levels do not gain much but hurt speed
static const int kCompressionLevel = 1;

// This enabled some specialized versions for RGB and RGBA
#define ENABLE_RGB_COLOR_FILTER
#define ENABLE_BAYER_FILTER

// Header definitions
#define ZPNG_HEADER_MAGIC 0xFBF8
#define ZPNG_VIDEO_HEADER_MAGIC 0xF8FB
#define ZPNG_HEADER_OVERHEAD_BYTES 8

// File format header
struct ZPNG_Header
{
    uint16_t Magic;
    uint16_t Width;
    uint16_t Height;
    uint8_t Channels;
    uint8_t BytesPerChannel;
};

#pragma clang optimize off

ZPNG_Context* ZPNG_AllocateCompressionContext()
{
    return (ZPNG_Context*)ZSTD_createCCtx();
}

void ZPNG_FreeCompressionContext(ZPNG_Context* context)
{
    ZSTD_freeCCtx((ZSTD_CCtx*)context);
}

void ZPNG_FreeDictionary(ZPNG_Dictionary* dict)
{
    ZSTD_freeCDict((ZSTD_CDict*)dict);
}

//------------------------------------------------------------------------------
// Image Processing

// Interleaving is a 1% compression win, and a 0.3% performance win: Not used.
// Splitting the data into blocks of 4 at a time actually reduces compression.

template<int kChannels>
static void PackAndFilter(
    const ZPNG_ImageData* imageData,
    uint8_t* output
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;

    for (unsigned y = 0; y < height; ++y)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            // For each channel:
            for (unsigned i = 0; i < kChannels; ++i)
            {
                uint8_t a = input[i];
                uint8_t d = a - prev[i];
                output[i] = d;
                prev[i] = a;
            }

            input += kChannels;
            output += kChannels;
        }
    }
}

template<int kChannels>
static void UnpackAndUnfilter(
    const uint8_t* input,
    ZPNG_ImageData* imageData
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    uint8_t* output = imageData->Buffer.Data;

    for (unsigned y = 0; y < height; ++y)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            // For each channel:
            for (unsigned i = 0; i < kChannels; ++i)
            {
                uint8_t d = input[i];
                uint8_t a = d + prev[i];
                output[i] = a;
                prev[i] = a;
            }

            input += kChannels;
            output += kChannels;
        }
    }
}

#ifdef ENABLE_BAYER_FILTER

// This should work well for RGGB and BGGR
static void PackAndFilterXGGY(
    const ZPNG_ImageData* imageData,
    uint8_t* output
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height / 4;
    uint8_t* output_r = output;
    uint8_t* output_b = output + planeBytes;
    uint8_t* output_g = output + planeBytes * 2;

    for (unsigned row = 0; row < height; row += 2)
    {
        uint8_t prev[2] = { 0, 0 };

        // even
        for (unsigned x = 0; x < width; x += 2)
        {
            uint8_t r = input[0];
            uint8_t g = input[1];

            r -= prev[0];
            g -= prev[1];

            prev[0] = input[0];
            prev[1] = input[1];

            *output_r++ = r;
            *output_g++ = g;

            input += 2;
        }

        prev[0] = prev[1] = 0;

        // odd
        for (unsigned x = 0; x < width; x += 2)
        {
            uint8_t g = input[0];
            uint8_t b = input[1];

            g -= prev[0];
            b -= prev[1];

            prev[0] = input[0];
            prev[1] = input[1];

            *output_b++ = b;
            *output_g++ = g;

            input += 2;
        }
    }
}

static void UnpackAndUnfilterXGGY(
    const uint8_t* input,
    ZPNG_ImageData* imageData
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    uint8_t* output = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height / 4;
    const uint8_t* input_r = input;
    const uint8_t* input_b = input + planeBytes;
    const uint8_t* input_g = input + planeBytes * 2;

    for (unsigned y = 0; y < height; y += 2)
    {
        uint8_t prev[2] = { 0, 0 };

        // even
        for (unsigned x = 0; x < width; x += 2)
        {
            uint8_t r = *input_r++;
            uint8_t g = *input_g++;

            // GB-RG filter from BCIF
            //r += g;

            r += prev[0];
            g += prev[1];

            output[0] = r;
            output[1] = g;

            prev[0] = r;
            prev[1] = g;

            output += 2;
        }

        prev[0] = prev[1] = 0;

        // odd
        for (unsigned x = 0; x < width; x += 2)
        {
            uint8_t g = *input_g++;
            uint8_t b = *input_b++;

            // GB-RG filter from BCIF
            //b += g;

            g += prev[0];
            b += prev[1];

            output[0] = g;
            output[1] = b;

            prev[0] = g;
            prev[1] = b;

            output += 2;
        }
    }
}
#endif

// returns the number of overflow bytes
template<int kChannels>
static int PackAndFilterVideo(
    const ZPNG_ImageData* refData,
    const ZPNG_ImageData* imageData,
    uint8_t* output
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;
    const uint8_t* ref = refData->Buffer.Data;

    uint8_t* outStart = output;
    unsigned overflowCount = 0;
    uint8_t* overflow = output + height * width * kChannels;

    for (unsigned y = 0; y < height; ++y)
    {
        for (unsigned x = 0; x < width; ++x)
        {
            // For each channel:
            for (unsigned i = 0; i < kChannels; ++i)
            {
                int diff = (int)input[i] - ref[i];
                if (diff > 127 || diff < -127) {
                    if (overflowCount == 1000) {
                        PackAndFilterXGGY(imageData, outStart);
                        return -1;
                    }
                    output[i] = 0x80;
                    *overflow = input[i];
                    ++overflow;
                    ++overflowCount;
                } else
                    output[i] = (int8_t)diff;
            }

            ref += kChannels;
            input += kChannels;
            output += kChannels;
        }
    }

    if (overflowCount != 0)
        printf("overflow: %d\n", overflowCount);

    return overflowCount;
}

template<int kChannels>
static void UnpackAndUnfilterVideo(
    const ZPNG_ImageData* refData,
    const uint8_t* input,
    ZPNG_ImageData* imageData
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* ref = refData->Buffer.Data;
    uint8_t* output = imageData->Buffer.Data;
    const uint8_t* overflow = input + height * width * kChannels;

    for (unsigned y = 0; y < height; ++y)
    {
        for (unsigned x = 0; x < width; ++x)
        {
            // For each channel:
            for (unsigned i = 0; i < kChannels; ++i)
            {
                if (input[i] == 0x80) {
                    output[i] = *overflow;
                    ++overflow;
                } else
                    output[i] = ref[i] + input[i];
            }

            ref += kChannels;
            input += kChannels;
            output += kChannels;
        }
    }
}

#ifdef ENABLE_RGB_COLOR_FILTER

template<>
void PackAndFilter<3>(
    const ZPNG_ImageData* imageData,
    uint8_t* output
    )
{
    static const unsigned kChannels = 3;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    uint8_t* output_y = output;
    uint8_t* output_u = output + planeBytes;
    uint8_t* output_v = output + planeBytes * 2;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t r = input[0];
            uint8_t g = input[1];
            uint8_t b = input[2];

            r -= prev[0];
            g -= prev[1];
            b -= prev[2];

            prev[0] = input[0];
            prev[1] = input[1];
            prev[2] = input[2];

            // GB-RG filter from BCIF
            uint8_t y = b;
            uint8_t u = g - b;
            uint8_t v = g - r;

            *output_y++ = y;
            *output_u++ = u;
            *output_v++ = v;

            input += kChannels;
        }
    }
}

template<>
void UnpackAndUnfilter<3>(
    const uint8_t* input,
    ZPNG_ImageData* imageData
    )
{
    static const unsigned kChannels = 3;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    uint8_t* output = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    const uint8_t* input_y = input;
    const uint8_t* input_u = input + planeBytes;
    const uint8_t* input_v = input + planeBytes * 2;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t y = *input_y++;
            uint8_t u = *input_u++;
            uint8_t v = *input_v++;

            // GB-RG filter from BCIF
            const uint8_t B = y;
            const uint8_t G = u + B;
            uint8_t r = G - v;
            uint8_t g = G;
            uint8_t b = B;

            r += prev[0];
            g += prev[1];
            b += prev[2];

            output[0] = r;
            output[1] = g;
            output[2] = b;

            prev[0] = r;
            prev[1] = g;
            prev[2] = b;

            output += kChannels;
        }
    }
}


// Version for RGBA (with alpha):

template<>
void PackAndFilter<4>(
    const ZPNG_ImageData* imageData,
    uint8_t* output
    )
{
    static const unsigned kChannels = 4;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    uint8_t* output_y = output;
    uint8_t* output_u = output + planeBytes;
    uint8_t* output_v = output + planeBytes * 2;
    uint8_t* output_a = output + planeBytes * 3;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t r = input[0];
            uint8_t g = input[1];
            uint8_t b = input[2];
            uint8_t a = input[3];

            r -= prev[0];
            g -= prev[1];
            b -= prev[2];
            a -= prev[3];

            prev[0] = input[0];
            prev[1] = input[1];
            prev[2] = input[2];
            prev[3] = input[3];

            // GB-RG filter from BCIF
            uint8_t y = b;
            uint8_t u = g - b;
            uint8_t v = g - r;

            *output_y++ = y;
            *output_u++ = u;
            *output_v++ = v;
            *output_a++ = a;

            input += kChannels;
        }
    }
}

template<>
void UnpackAndUnfilter<4>(
    const uint8_t* input,
    ZPNG_ImageData* imageData
    )
{
    static const unsigned kChannels = 4;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    uint8_t* output = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    const uint8_t* input_y = input;
    const uint8_t* input_u = input + planeBytes;
    const uint8_t* input_v = input + planeBytes * 2;
    const uint8_t* input_a = input + planeBytes * 3;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t y = *input_y++;
            uint8_t u = *input_u++;
            uint8_t v = *input_v++;
            uint8_t a = *input_a++;

            // GB-RG filter from BCIF
            const uint8_t B = y;
            const uint8_t G = u + B;
            uint8_t r = G - v;
            uint8_t g = G;
            uint8_t b = B;

            r += prev[0];
            g += prev[1];
            b += prev[2];
            a += prev[3];

            output[0] = r;
            output[1] = g;
            output[2] = b;
            output[3] = a;

            prev[0] = r;
            prev[1] = g;
            prev[2] = b;
            prev[3] = a;

            output += kChannels;
        }
    }
}

#endif


#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// API



unsigned ZPNG_MaximumBufferSize(
    const ZPNG_ImageData* imageData
)
{
    const unsigned pixelCount = imageData->WidthPixels * imageData->HeightPixels;
    const unsigned pixelBytes = (imageData->BytesPerChannel > 8) ? imageData->Channels : imageData->BytesPerChannel * imageData->Channels;
    const unsigned byteCount = pixelBytes * pixelCount + 1000;
    const unsigned maxOutputBytes = (unsigned)ZSTD_compressBound(byteCount);
    return ZPNG_HEADER_OVERHEAD_BYTES + maxOutputBytes;
}

ZPNG_Buffer ZPNG_Compress(
    const ZPNG_ImageData* imageData,
    ZPNG_Context* context,
    ZPNG_Dictionary** dictionary
)
{
    ZPNG_Buffer buffer;
    buffer.Bytes = 0;
    buffer.Data = nullptr;
    ZPNG_CompressVideoToBuffer(nullptr, imageData, &buffer, context, dictionary);

    return buffer;
}

int ZPNG_CompressToBuffer(
    const ZPNG_ImageData* imageData,
    ZPNG_Buffer* bufferOutput,
    ZPNG_Context* context,
    ZPNG_Dictionary** dictionary
)
{
    return ZPNG_CompressVideoToBuffer(0, imageData, bufferOutput, context, dictionary);
}

int ZPNG_CompressVideoToBuffer(
    const ZPNG_ImageData* refData,
    const ZPNG_ImageData* imageData,
    ZPNG_Buffer* bufferOutput,
    ZPNG_Context* context,
    ZPNG_Dictionary** dictionary
)
{
    uint8_t* packing = nullptr;
    uint8_t* output = nullptr;

    const unsigned pixelCount = imageData->WidthPixels * imageData->HeightPixels;
    const unsigned pixelBytes = (imageData->BytesPerChannel > 8) ? imageData->Channels : imageData->BytesPerChannel * imageData->Channels;
    const unsigned byteCount = pixelBytes * pixelCount;

    // FIXME: One day add support for other formats
    if (pixelBytes > 8) {
        return 0;
    }

    // Space for packing
    packing = (uint8_t*)calloc(1, byteCount + 1000);

    if (!packing) {
ReturnResult:
        if (bufferOutput->Data != output && output) {
            free(output);
        }
        free(packing);
        return 0;
    }

    const unsigned maxOutputBytes = (unsigned)ZSTD_compressBound(byteCount);
    if (bufferOutput->Bytes == 0) {
        output = (uint8_t*)calloc(1, ZPNG_HEADER_OVERHEAD_BYTES + maxOutputBytes);
    } else if (bufferOutput->Bytes < ZPNG_HEADER_OVERHEAD_BYTES + maxOutputBytes) {
        return 0;
    } else {
        output = bufferOutput->Data;
    }

    if (!output) {
        goto ReturnResult;
    }

    int overflowCount = 0;

    // Pass 1: Pack and filter data.
    if (refData)
    {
        switch (pixelBytes)
        {
        case 1: {
            overflowCount = PackAndFilterVideo<1>(refData, imageData, packing);
            // ZPNG_ImageData out = *refData;
            // out.Buffer.Data = (uint8_t*)calloc(1, byteCount);
            // UnpackAndUnfilterVideo<1>(refData, packing, &out);
            // if (memcmp(imageData->Buffer.Data, out.Buffer.Data, out.Buffer.Bytes) != 0) {
            //     printf("unpacking doesn't match packing\n");
            // }
            break; }
        case 2:
            overflowCount = PackAndFilterVideo<2>(refData, imageData, packing);
            break;
        case 3:
            overflowCount = PackAndFilterVideo<3>(refData, imageData, packing);
            break;
        case 4:
            overflowCount = PackAndFilterVideo<4>(refData, imageData, packing);
            break;
        case 5:
            overflowCount = PackAndFilterVideo<5>(refData, imageData, packing);
            break;
        case 6:
            overflowCount = PackAndFilterVideo<6>(refData, imageData, packing);
            break;
        case 7:
            overflowCount = PackAndFilterVideo<7>(refData, imageData, packing);
            break;
        case 8:
            overflowCount = PackAndFilterVideo<8>(refData, imageData, packing);
            break;
        }
    } else {
        if (imageData->BytesPerChannel > 8) {
            PackAndFilterXGGY(imageData, packing);
        } else {
            switch (pixelBytes)
            {
            case 1:
                PackAndFilter<1>(imageData, packing);
                break;
            case 2:
                PackAndFilter<2>(imageData, packing);
                break;
            case 3:
                PackAndFilter<3>(imageData, packing);
                break;
            case 4:
                PackAndFilter<4>(imageData, packing);
                break;
            case 5:
                PackAndFilter<5>(imageData, packing);
                break;
            case 6:
                PackAndFilter<6>(imageData, packing);
                break;
            case 7:
                PackAndFilter<7>(imageData, packing);
                break;
            case 8:
                PackAndFilter<8>(imageData, packing);
                break;
            }
        }
    }

    // Pass 2: Compress the packed/filtered data.
    size_t result;
    if (context)
    {
        if (*dictionary == nullptr)
        {
            char* dictBuf = (char*)malloc(100000);
            size_t* sampleSizes = (size_t*)malloc(imageData->HeightPixels * 8 * sizeof(size_t));
            for (uint i = 0; i < imageData->HeightPixels * 8; ++i)
                sampleSizes[i] = byteCount / imageData->HeightPixels / 8;
            ZDICT_cover_params_t params = {32, 8, 0, 1, {kCompressionLevel, 0, 0}};
            size_t actualSize = ZDICT_trainFromBuffer_cover(dictBuf, 100000, packing, sampleSizes, imageData->HeightPixels * 8, params);
            free(sampleSizes);
            *dictionary = (void*)ZSTD_createCDict(dictBuf, actualSize, kCompressionLevel);
            free(dictBuf);
        }
        result = ZSTD_compress_usingCDict(
            (ZSTD_CCtx*)context,
            output + ZPNG_HEADER_OVERHEAD_BYTES,
            maxOutputBytes,
            packing,
            byteCount + ((overflowCount >= 0) ? overflowCount : 0),
            (ZSTD_CDict*)*dictionary);
    } else
    {
        result = ZSTD_compress(
            output + ZPNG_HEADER_OVERHEAD_BYTES,
            maxOutputBytes,
            packing,
            byteCount + ((overflowCount >= 0) ? overflowCount : 0),
            kCompressionLevel);
    }

    if (ZSTD_isError(result)) {
        goto ReturnResult;
    }

    // Write header

    ZPNG_Header* header = (ZPNG_Header*)output;
    header->Magic = (refData && overflowCount >= 0) ? ZPNG_VIDEO_HEADER_MAGIC : ZPNG_HEADER_MAGIC;
    header->Width = (uint16_t)imageData->WidthPixels;
    header->Height = (uint16_t)imageData->HeightPixels;
    header->Channels = (uint8_t)imageData->Channels;
    header->BytesPerChannel = (uint8_t)imageData->BytesPerChannel;

    bufferOutput->Data = output;
    bufferOutput->Bytes = ZPNG_HEADER_OVERHEAD_BYTES + (unsigned)result;

    goto ReturnResult;
}

ZPNG_ImageData ZPNG_Decompress(
    ZPNG_Buffer buffer
)
{
    return ZPNG_DecompressVideo(nullptr, buffer);
}

ZPNG_ImageData ZPNG_DecompressVideo(
    const ZPNG_ImageData* refData,
    ZPNG_Buffer buffer
)
{
    uint8_t* packing = nullptr;
    uint8_t* output = nullptr;

    ZPNG_ImageData imageData;
    imageData.Buffer.Data = nullptr;
    imageData.Buffer.Bytes = 0;
    imageData.BytesPerChannel = 0;
    imageData.Channels = 0;
    imageData.HeightPixels = 0;
    imageData.StrideBytes = 0;
    imageData.WidthPixels = 0;
    imageData.IsIFrame = 1;

    if (!buffer.Data || buffer.Bytes < ZPNG_HEADER_OVERHEAD_BYTES) {
ReturnResult:
        if (imageData.Buffer.Data != output) {
            free(output);
        }
        free(packing);
        return imageData;
    }

    const ZPNG_Header* header = (const ZPNG_Header*)buffer.Data;
    if (refData == nullptr) {
        if (header->Magic != ZPNG_HEADER_MAGIC) {
            goto ReturnResult;
        }
    } else {
        if (header->Magic != ZPNG_HEADER_MAGIC) {
            if (header->Magic == ZPNG_VIDEO_HEADER_MAGIC)
                imageData.IsIFrame = 0;
            else
                goto ReturnResult;
        }
    }

    imageData.WidthPixels = header->Width;
    imageData.HeightPixels = header->Height;
    imageData.Channels = header->Channels;
    imageData.BytesPerChannel = header->BytesPerChannel;
    imageData.StrideBytes = imageData.WidthPixels * imageData.Channels;

    const unsigned pixelCount = imageData.WidthPixels * imageData.HeightPixels;
    const unsigned pixelBytes = (imageData.BytesPerChannel > 8) ? imageData.Channels : imageData.BytesPerChannel * imageData.Channels;
    const unsigned byteCount = pixelBytes * pixelCount;

    // Space for packing
    packing = (uint8_t*)calloc(1, byteCount + 1000);

    if (!packing) {
        goto ReturnResult;
    }

    // Stage 1: Decompress back to packing buffer

    const size_t result = ZSTD_decompress(
        packing,
        byteCount + 1000,
        buffer.Data + ZPNG_HEADER_OVERHEAD_BYTES,
        buffer.Bytes - ZPNG_HEADER_OVERHEAD_BYTES);

    if (ZSTD_isError(result)) {
        goto ReturnResult;
    }

    // Stage 2: Unpack/Unfilter

    // Space for output
    output = (uint8_t*)calloc(1, byteCount);

    if (!output) {
        goto ReturnResult;
    }

    imageData.Buffer.Data = output;
    imageData.Buffer.Bytes = byteCount;

    if (refData && !imageData.IsIFrame) {
            switch (pixelBytes)
            {
            case 1:
                UnpackAndUnfilterVideo<1>(refData, packing, &imageData);
                break;
            case 2:
                UnpackAndUnfilterVideo<2>(refData, packing, &imageData);
                break;
            case 3:
                UnpackAndUnfilterVideo<3>(refData, packing, &imageData);
                break;
            case 4:
                UnpackAndUnfilterVideo<4>(refData, packing, &imageData);
                break;
            case 5:
                UnpackAndUnfilterVideo<5>(refData, packing, &imageData);
                break;
            case 6:
                UnpackAndUnfilterVideo<6>(refData, packing, &imageData);
                break;
            case 7:
                UnpackAndUnfilterVideo<7>(refData, packing, &imageData);
                break;
            case 8:
                UnpackAndUnfilterVideo<8>(refData, packing, &imageData);
                break;
            }
    } else {
        if (imageData.BytesPerChannel > 8) {
            UnpackAndUnfilterXGGY(packing, &imageData);
        } else {
            switch (pixelBytes)
            {
            case 1:
                UnpackAndUnfilter<1>(packing, &imageData);
                break;
            case 2:
                UnpackAndUnfilter<2>(packing, &imageData);
                break;
            case 3:
                UnpackAndUnfilter<3>(packing, &imageData);
                break;
            case 4:
                UnpackAndUnfilter<4>(packing, &imageData);
                break;
            case 5:
                UnpackAndUnfilter<5>(packing, &imageData);
                break;
            case 6:
                UnpackAndUnfilter<6>(packing, &imageData);
                break;
            case 7:
                UnpackAndUnfilter<7>(packing, &imageData);
                break;
            case 8:
                UnpackAndUnfilter<8>(packing, &imageData);
                break;
            }
        }
    }

    goto ReturnResult;
}

void ZPNG_Free(
    ZPNG_Buffer* buffer
)
{
    if (buffer && buffer->Data)
    {
        free(buffer->Data);
        buffer->Data = nullptr;
        buffer->Bytes = 0;
    }
}


#ifdef __cplusplus
}
#endif
