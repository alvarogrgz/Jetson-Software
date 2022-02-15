/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2017
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf {
namespace utility {
namespace commoninterfaces {

/*
 * @brief This interface should be implemented by all classes representing any kind of device. The
 *        class constructor should only initialize the appropriate fields, but should not
 *        communicate with the device (yet). It is allowed to call `deinitialize` in the
 *        destructor, however it is STRONGLY discouraged to call `initialize` in the constructor.
 *        Some trivial exceptions might be allowed.
 */
class IInitializable {
 public:
    virtual ~IInitializable() = default;

    /*
     * @brief The method `initialize` should effectively configure the device being represented
     *        and make it usable by further calls to the particular interface. Example:
     *
     *            std::unique_ptr<MyDevice> dev(new MyDevice("/dev/ttyUSB0"));
     *            // If needed do some additional work before using device
     *            configureEnvironment(myEnvSettings);
     *            dev->initialize();
     *            int result = dev->getResult();
     *            logger->info("Obtained result: {}", result);
     *
     * @return True upon success or if the device is already initialized.
     * @return False upon failure (e.g. device not connected)
     */
    virtual bool initialize() = 0;
    /*
     * @brief The method `deinitialize` should effectively release the considered device's
     *        resources (e.g. close the files, streams, whatever) but should not erase the
     *        variables initialized in the constructor. It should be possible (and it should be
     *        tested) to call initialize/deinitialize on the same object multiple times. Example:
     *
     *            // After we have used the device for some time ...
     *            dev->deinitialize();
     *            changeEnvironmentSettings(newEnvSettings);
     *            dev->initialize();
     *            while(!stop_) {
     *                logger->info("Obtained result: {}", dev->getResult());
     *            }
     *            dev->deinitialize();
     *           // Do something else ...
     *
     * @return True upon success or if the device was not previously initialized
     * @return False upon failure (e.g. fail to communicate with the device)
     */
    virtual bool deinitialize() = 0;
};

}  // namespace commoninterfaces
}  // namespace utility
}  // namespace crf
