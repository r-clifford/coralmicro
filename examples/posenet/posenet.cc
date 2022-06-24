#include "libs/tensorflow/posenet.h"

#include "libs/base/filesystem.h"
#include "libs/camera/camera.h"
#include "libs/tensorflow/posenet_decoder_op.h"
#include "libs/tpu/edgetpu_manager.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/all_ops_resolver.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "third_party/tflite-micro/tensorflow/lite/micro/micro_interpreter.h"

// Runs pose estimation using PoseNet, running on the Edge TPU.
// Scores and keypoint data is printed to the serial console.

namespace {
constexpr int kModelArenaSize = 1 * 1024 * 1024;
constexpr int kExtraArenaSize = 1 * 1024 * 1024;
constexpr int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
STATIC_TENSOR_ARENA_IN_SDRAM(tensor_arena, kTensorArenaSize);
constexpr char kModelPath[] =
    "/models/posenet_mobilenet_v1_075_324_324_16_quant_decoder_edgetpu.tflite";
constexpr char kTestInputPath[] = "/models/posenet_test_input_324.bin";
}  // namespace

extern "C" void app_main(void* param) {
    tflite::MicroErrorReporter error_reporter;
    TF_LITE_REPORT_ERROR(&error_reporter, "Posenet!");

    // Turn on the TPU and get it's context.
    auto tpu_context = coral::micro::EdgeTpuManager::GetSingleton()->OpenDevice(
        coral::micro::PerformanceMode::kMax);
    if (!tpu_context) {
        printf("ERROR: Failed to get EdgeTpu context\r\n");
        vTaskSuspend(nullptr);
    }

    // Reads the model and checks version.
    std::vector<uint8_t> posenet_tflite;
    if (!coral::micro::filesystem::ReadFile(kModelPath, &posenet_tflite)) {
        TF_LITE_REPORT_ERROR(&error_reporter, "Failed to load model!");
        vTaskSuspend(nullptr);
    }
    auto* model = tflite::GetModel(posenet_tflite.data());
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        TF_LITE_REPORT_ERROR(&error_reporter,
                             "Model schema version is %d, supported is %d",
                             model->version(), TFLITE_SCHEMA_VERSION);
        vTaskSuspend(nullptr);
    }

    // Creates a micro interpreter.
    tflite::MicroMutableOpResolver<2> resolver;
    resolver.AddCustom(coral::micro::kCustomOp,
                       coral::micro::RegisterCustomOp());
    resolver.AddCustom(coral::kPosenetDecoderOp,
                       coral::RegisterPosenetDecoderOp());
    auto interpreter = tflite::MicroInterpreter{
        model, resolver, tensor_arena, kTensorArenaSize, &error_reporter};
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(&error_reporter, "AllocateTensors failed.");
        vTaskSuspend(nullptr);
    }
    auto* posenet_input = interpreter.input(0);

    // Runs posenet on a test image.
    printf("Getting outputs for posenet test input\r\n");
    std::vector<uint8_t> posenet_test_input_bin;
    if (!coral::micro::filesystem::ReadFile(kTestInputPath,
                                            &posenet_test_input_bin)) {
        TF_LITE_REPORT_ERROR(&error_reporter, "Failed to load test input!");
        vTaskSuspend(nullptr);
    }
    if (posenet_input->bytes != posenet_test_input_bin.size()) {
        TF_LITE_REPORT_ERROR(
            &error_reporter,
            "Input tensor length doesn't match canned input\r\n");
        vTaskSuspend(nullptr);
    }
    memcpy(tflite::GetTensorData<uint8_t>(posenet_input),
           posenet_test_input_bin.data(), posenet_test_input_bin.size());
    if (interpreter.Invoke() != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(&error_reporter, "Invoke failed.");
        vTaskSuspend(nullptr);
    }
    auto test_image_output = coral::micro::tensorflow::GetPosenetOutput(
        &interpreter, /*threshold=*/0.5, /*print=*/true);

    // Starts the camera for live poses.
    coral::micro::CameraTask::GetSingleton()->SetPower(true);
    coral::micro::CameraTask::GetSingleton()->Enable(
        coral::micro::camera::Mode::STREAMING);

    printf("Starting live posenet\r\n");
    auto model_height = posenet_input->dims->data[1];
    auto model_width = posenet_input->dims->data[2];
    for (;;) {
        coral::micro::camera::FrameFormat fmt{
            /*fmt=*/coral::micro::camera::Format::RGB,
            /*filter=*/coral::micro::camera::FilterMethod::BILINEAR,
            /*rotation=*/coral::micro::camera::Rotation::k0,
            /*width=*/model_width,
            /*height=*/model_height,
            /*preserve_ratio=*/false,
            /*buffer=*/tflite::GetTensorData<uint8_t>(posenet_input)};
        if (!coral::micro::CameraTask::GetFrame({fmt})) {
            TF_LITE_REPORT_ERROR(&error_reporter,
                                 "Failed to get image from camera.");
            break;
        }
        if (interpreter.Invoke() != kTfLiteOk) {
            TF_LITE_REPORT_ERROR(&error_reporter, "Invoke failed.");
            break;
        }
        auto output = coral::micro::tensorflow::GetPosenetOutput(
            &interpreter, /*threshold=*/0.5, /*print=*/true);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    coral::micro::CameraTask::GetSingleton()->SetPower(false);
    vTaskSuspend(nullptr);
}