## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

SET(PACKAGE_VERSION 4.4.0)

SET(PACKAGE_VERSION_EXACT 0)
SET(PACKAGE_VERSION_COMPATIBLE 0)

IF (PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
  SET(PACKAGE_VERSION_EXACT 1)
  SET(PACKAGE_VERSION_COMPATIBLE 1)
ENDIF()

IF (PACKAGE_FIND_VERSION_MAJOR EQUAL 4 AND PACKAGE_FIND_VERSION VERSION_LESS PACKAGE_VERSION)
  SET(PACKAGE_VERSION_COMPATIBLE 1)
ENDIF()
