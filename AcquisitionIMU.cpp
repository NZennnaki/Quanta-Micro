//
// Created by Yanis Filippi on 26/04/2022.
//

#include <cstdio>
#include <string>
#include <chrono>
#include <cmath>
#include <ctime>
#include <csignal>

// sbgECom
extern "C" {
#include <sbgCommon.h>
#include <version/sbgVersion.h>
#include <sbgEComLib.h>
}

typedef std::chrono::steady_clock clk;
typedef std::chrono::duration<int64_t , std::micro> duration_us;
typedef std::chrono::time_point<clk, duration_us> timePoint_us;

struct callbackArgument
{
    clk::time_point reference;
    FILE *pFile = nullptr;
};

int flagEndLoop = 0;

/**
 * @brief Signal handler for SIGINT & SIGTERM signals.
 * @param signal Signal code.
 */
void signalHandler(int signal)
{
    printf("Process received signal ");
    switch (signal)
    {
        case SIGINT:  printf("SIGINT");  break;
        case SIGTERM: printf("SIGTERM"); break;
        default:      printf("Unknown"); break;
    }
    printf("\n");
    flagEndLoop = 1;
}

/**
 * @brief Prints a help message to the stdout console.
 */
void printHelpMessage()
{
    printf("Usage:      $ ./SoftIMU <PORT> <BAUD> <OUTPUT_FILE> <FREQUENCY>\n");
    printf("Example:    $ ./SoftIMU /dev/ttyUSB0 115200 output.txt 10\n");
    printf("\n");
    printf("PORT:           [string]\n");
    printf("\tSerial port on which the IMU is plugged.\n");
    printf("\n");
    printf("BAUD:           [int]\n");
    printf("\tBaud rate at which open the serial connexion. Normally at 115200 bauds\n");
    printf("\n");
    printf("OUTPUT_FILE:    [string]\n");
    printf("\tPath (absolute or relative, absolute is recommended) to the desired output file. If none exists "
           "the file will be generated.\n");
    printf("\n");
    printf("FREQUENCY:      [int]\n");
    printf("\tFrequency in Hertz (Hz) at which data should be collected. Possible values are:\n");
    printf("\t\t- 200 Hz\n");
    printf("\t\t- 100 Hz\n");
    printf("\t\t- 50  Hz\n");
    printf("\t\t- 40  Hz\n");
    printf("\t\t- 25  Hz\n");
    printf("\t\t- 20  Hz\n");
    printf("\t\t- 10  Hz\n");
    printf("\t\t- 5   Hz\n");
    printf("\t\t- 1   Hz\n");
}

/**
 * @brief Callback called by the sbgECom library every time a log is received
 * @param pHandle sbgECom handle. Unused.
 * @param msgClass Message class. Only ECOM_0 messages are treated.
 * @param msg Message Id. Only EKF_EULER messages are treated.
 * @param pLogData Pointer to the log data.
 * @param pUserArg User argument: Pointer to a callbackArguments struct.
 * @return Error code.
 */
SbgErrorCode logCallback(SbgEComHandle *pHandle, SbgEComClass msgClass,
                         SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
    clk::time_point timeReference = ((callbackArgument *)pUserArg)->reference;
    clk::time_point current = clk::now();
    int64_t elapsedSinceRef = std::chrono::duration_cast<duration_us>(current - timeReference).count();
    FILE *pOpenFile = ((callbackArgument *)pUserArg)->pFile;

    SBG_UNUSED_PARAMETER(pHandle);

    // Filter log
    if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        switch (msg)
        {
            case SBG_ECOM_LOG_EKF_EULER:
                // Write to file
                fwrite(&elapsedSinceRef, sizeof(int64_t), 1, pOpenFile);
                fwrite(pLogData->ekfEulerData.euler, sizeof(float), 3, pOpenFile);
                fflush(pOpenFile);
                break;
            default:
                break;
        }
    }

    return SBG_NO_ERROR;
}

int main(int argc, char *argv[])
{
    char *port, *outputFilePath;
    uint32_t baud;
    SbgEComOutputMode frequency;
    SbgErrorCode errorCode = SBG_NO_ERROR;
    SbgInterface sbgInterface;
    SbgEComHandle comHandle;
    auto *pUserArg = new callbackArgument;
    time_t unixTimestamp;

    // Setup signal handling
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Command line arguments
    if (argc != 5)
    {
        printf("Error: Wrong command line arguments.\n");
        printHelpMessage();
        return 1;
    }
    port = argv[1];
    baud = (uint32_t)atoi(argv[2]);
    outputFilePath = argv[3];
    switch (atoi(argv[4]))
    {
        case 200:   frequency = SBG_ECOM_OUTPUT_MODE_MAIN_LOOP; break;
        case 100:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_2;     break;
        case  50:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_4;     break;
        case  40:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_5;     break;
        case  25:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_8;     break;
        case  20:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_10;    break;
        case  10:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_20;    break;
        case   5:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_40;    break;
        case   1:   frequency = SBG_ECOM_OUTPUT_MODE_DIV_200;   break;
        default: printf("Error: invalid frequency.\n"); printHelpMessage(); return 1;
    }

    // Wait for next timestamp second to begin to fetch the time reference
    time (&unixTimestamp);
    while (time(NULL) <= unixTimestamp) {}
    time(&unixTimestamp);
    pUserArg->reference = clk::now();

    // Open (or create if necessary) the output file
    pUserArg->pFile = fopen(outputFilePath, "w");

    // Write header
    fwrite(&unixTimestamp, sizeof(time_t), 1, pUserArg->pFile);
    fflush(pUserArg->pFile);

    // Open Serial port
    errorCode = sbgInterfaceSerialCreate(&sbgInterface, port, baud);
    if (errorCode != SBG_NO_ERROR)
    {
        printf("Error while opening Serial port. Encountered %s.\n", sbgErrorCodeToString(errorCode));
        return 1;
    }

    // Initialise library
    errorCode = sbgEComInit(&comHandle, &sbgInterface);
    if (errorCode != SBG_NO_ERROR)
    {
        printf("Error initialising sbg library. Encountered %s.\n", sbgErrorCodeToString(errorCode));
        return 1;
    }

    // Configure logs
    errorCode = sbgEComCmdOutputSetConf(
            &comHandle,
            SBG_ECOM_OUTPUT_PORT_A,
            SBG_ECOM_CLASS_LOG_ECOM_0,
            SBG_ECOM_LOG_EKF_EULER,
            frequency);
    if (errorCode != SBG_NO_ERROR)
    {
        printf("Error while configuring LOGS. Encountered %s.\n", sbgErrorCodeToString(errorCode));
        return 1;
    }

    // Set log callback function
    sbgEComSetReceiveLogCallback(&comHandle, logCallback, pUserArg);

    for (;;)
    {
        // Try to read a frame
        errorCode = sbgEComHandle(&comHandle);

        // Test if we have to release some CPU (no frame received)
        if (errorCode == SBG_NOT_READY) {sbgSleep(1);}
        else {SBG_LOG_ERROR(errorCode, "Unable to process incoming sbgECom logs");}

        // Check flag
        if (flagEndLoop) break;
    }

    printf("Exiting.\n");
    fclose(pUserArg->pFile);
    sbgEComClose(&comHandle);
    sbgInterfaceDestroy(&sbgInterface);
    return 0;
}
