/*
 * Copyright (c) 2012-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/*
 * ======== configif.h ========
 *
 * Configuration Manager Interface functions
 *
 */
/**
 *  @file  ti/ndk/inc/nettools/inc/configif.h
 *
 *  @addtogroup ti_ndk_inc_nettools_inc__Cfg Configuration Manager
 *
 *  @brief      The Configuration Manager is a collection of API
 *              functions to help you create and manipulate a
 *              configuration. The manager API is independent of the
 *              configuration specification.
 */

#ifndef _CONFIGIF_H
#define _CONFIGIF_H

/*! @ingroup ti_ndk_inc_nettools_inc__Cfg */
/*! @{ */

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief      Create a new configuration
 *
 *  Creates a configuration handle that can be used with other
 *  configuration functions. The new handle defaults to the inactive
 *  state (see CfgExecute()).
 *
 *  @return     Success: handle to new configuration
 *  @return     Failure: NULL
 *
 *  @sa CfgFree()
 */
extern void *CfgNew();

/*!
 *  @brief      Destroy a new configuration
 *
 *  Destroys a configuration. Unloads and frees all configuration
 *  entries and frees the configuration handle. After this call, the
 *  configuration handle @c hCfg is invalid.
 *
 *  @param[in]  hCfg        Handle to configuration
 *
 *  @sa CfgNew()
 */
extern void CfgFree( void *hCfg );

/*!
 *  @brief      Set default configuration handle
 *
 *  This function sets the current default configuration handle to
 *  that specified in @c hCfg. The default handle is used in any
 *  function that takes a @c hCfg parameter, when the specified
 *  parameter is NULL. At initialization, there is no default
 *  configuration. It must be allocated by CfgNew() and then specified
 *  via CfgSetDefault(). Normally, the default configuration is
 *  reserved for system use. The default configuration handle should
 *  not be freed until it is cleared by calling @c CfgSetDefault(NULL).
 *
 *  @param[in]  hCfg        Handle to configuration to set as default, or
 *                          NULL to clear default.
 *
 *  @sa CfgGetDefault()
 */
extern void CfgSetDefault( void *hCfg );

/*!
 *  @brief      Get default configuration handle
 *
 *  This function returns the current default configuration
 *  handle. The default handle is used in any function that takes a
 *  hCfg parameter, when the specified parameter is NULL. At
 *  initialization, there is no default configuration. It must be
 *  allocated by CfgNew() and then specified via
 *  CfgSetDefault().
 *
 *  @remark     Normally, the default configuration is reserved
 *              for system use.
 *
 *  @sa CfgSetDefault()
 */
extern void *CfgGetDefault();

/*!
 *  @brief      Load a config from a linear memory block
 *
 *  @param[in,out] hCfg     Handle to configuration
 *  @param[in]  Size        Size of memory block to load
 *  @param[in]  pData       Pointer to memory block to load
 *
 *  @remark     The configuration system features the ability for the
 *              manager to convert a configuration database to a
 *              linear block of memory for storage in non-volatile
 *              memory. The configuration can then be converted back
 *              on reboot.
 *
 *  @remark     This function converts a linear block of memory to a
 *              configuration by loading each configuration entry it
 *              finds in the coded data block. Note that CfgLoad() can
 *              be used to load entries into a configuration that
 *              already has pre-existing entries, but the method of
 *              entry is not preserved (see Mode parameter of
 *              CfgAddEntry() ). To ensure that the resulting
 *              configuration exactly matches the one converted with
 *              CfgSave() , this function should only be called on an
 *              empty configuration handle.
 *
 *  @return     Success: > 0 - number of bytes loaded
 *  @return     Failure: < 0, @cfgerrorcodes
 *
 *  @sa CfgSave()
 */
extern int  CfgLoad( void *hCfg, int Size, unsigned char *pData );

/*!
 *  @brief      Save a config to a linear memory block
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in,out] pSize    Pointer to size of memory block
 *  @param[in]  pData       Pointer to memory block to load
 *
 *  @remark     One of the features of the configuration system is the
 *              ability for the manager to convert a configuration
 *              database to a linear block of memory for storage in
 *              non-volatile memory. The configuration can then be
 *              converted back on reboot.
 *
 *  @remark     This function saves the contents of the configuration
 *              specified by @c hCfg into the linear block of memory
 *              pointed to by @c pData.
 *
 *  @remark     The size of the data buffer is initially pointed to by the
 *              @c pSize parameter. If this size value pointed to by this
 *              pointer is zero (@c pSize cannot itself be NULL), the
 *              function does not attempt to save the configuration
 *              but rather calculates the size required and writes
 *              this value to the location specified by @c pSize. In
 *              fact, any time the value at pSize is less than the
 *              size required to store the configuration, the function
 *              returns 0 and the value at @c pSize is set to the size
 *              required to store the data.
 *
 *  @remark     The @c pData parameter points to the data buffer to
 *              receive the configuration information. This pointer
 *              can be NULL if @c *pSize is zero. Note that the pointer
 *              @c pSize must always be valid.
 *
 *  @return     Success: > 0 - number of bytes written
 *  @return     Failure:
 *                * 0 - size error (@c pSize will be set to the required size)
 *                * < 0, @cfgerrorcodes
 *
 *  @sa CfgSave()
 */
extern int  CfgSave( void *hCfg, int *pSize, unsigned char *pData );

/*!
 *  @brief      Set the Tag initialization and shutdown order on execute
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  Tags        Number of Tag values in @c pOpenOrder and
 *                          @c pCloseOder.  Must match #CFGTAG_MAX.
 *  @param[in]  pOpenOrder  Pointer to array of Tag values in init order
 *  @param[in]  pCloseOrder Pointer to array of Tag values in shutdown order
 *
 *  @remark     The configuration API has no knowledge of the
 *              configuration database specification. Thus, it has no
 *              concept of a priority in loading and unloading
 *              configuration entries. The default order for both
 *              loading and unloading is by ascending tag value.
 *
 *  @remark     You may require that the application specify the exact
 *              order in which entries should be initialized
 *              (specified in @c pOpenOrder) and shut down (specified
 *              in @c pCloseOder).
 *
 *  @remark     Both arrays must be provided - even if they are identical
 *              pointers.
 *
 *  @remark     The number of elements in each array is specified by the
 *              Tags parameter. This must exactly match the maximum
 *              number of tags in the system defined by CFGTAG_MAX. An
 *              entry of 0 in either order array is used as a
 *              placeholder for tags that have not yet been defined.
 *
 *  @return     Success: 0
 *  @return     Failure: < 0, @cfgerrorcodes
 *
 *  @sa CfgExecute()
 */
extern int CfgSetExecuteOrder( void *hCfg, uint32_t Tags,
                        uint32_t *pOpenOrder, uint32_t *pCloseOrder );

/*!
 *  @brief      Set the execution state (active/inactive) of the config
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  fExecute    Desired execute state (1 = active)
 *
 *  @remark     When a configuration is first created, it is in an
 *              inactive state, so changes to the configuration are
 *              not reflected by changes to the system.
 *
 *  @remark     Executing the configuration (setting @c fExecute to 1)
 *              causes all current entries in the configuration to be
 *              loaded, and any further changes in the configuration
 *              to be immediately reflected in the system.
 *
 *  @remark     Disabling execution of the configuration (setting
 *              @c fExecute to 0) causes all configuration entries to be
 *              unloaded from the system (note that they are not
 *              removed from the configuration). Any further changes
 *              to the configuration are not reflected by changes to
 *              the system.
 *
 *  @return     Success: 0
 *  @return     Failure: < 0, @cfgerrorcodes
 *
 *  @sa CfgNew()
 */
extern int  CfgExecute( void *hCfg, uint32_t fExecute );

/*!
 *  @brief      Set service callback for configuration Tag
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  Tag         Tag value to set callback for
 *  @param[in]  pCb         Pointer to service callback function
 *
 *  @remark     To give the configuration the ability to be active - i.e.,
 *              to make real-time changes to the system as the
 *              configuration changes, the configuration manager must
 *              have the ability to make changes to the system. To
 *              enable this in a generic fashion, the configuration
 *              manager allows for the installation of service
 *              callback functions for each configuration tag value.
 *
 *  @remark     This function sets the service function for the provided
 *              @c Tag. Service function pointers default to NULL, and
 *              when they are NULL, no service is performed for the
 *              configuration entry (it becomes information data
 *              only).
 *
 *  @remark     When invoked, the service callback function is passed
 *              information about the affected entry. The callback
 *              function is defined as:
 *
 *  @code
 *      int CbSrv(void *hCfg, uint32_t Tag, uint32_t Item, uint32_t Op,
 *          void *hCfgEntry);
 *  @endcode
 *
 *  @remark     where:
 *                * hCfg: Pointer to Config
 *                * Tag: Tag value of entry changed
 *                * Item: Item value of entry changed
 *                * Op: Operation (#CFGOP_ADD or #CFGOP_REMOVE)
 *                * hCfgEntry: Non-Referenced pointer to entry added or removed
 *
 *  @remark     @c pCb should return 1 on success, 0 on pass, and <0 on error.
 *
 *  @note       The configuration entry handle passed to the callback
 *              function is not referenced, as its scope expires when
 *              the callback function returns.
 *
 *  @return     Success: 0
 *  @return     Failure: < 0, @cfgerrorcodes
 *
 *  @sa CfgNew()
 */
extern int  CfgSetService( void *hCfg, uint32_t Tag,
                           int (*pCb) (void *, uint32_t, uint32_t, uint32_t,
                           void *) );

#define CFGOP_REMOVE    0       /**< Item was removed */
#define CFGOP_ADD       1       /**< Item was added */

/*!
 *  @brief      Add configuration entry to configuration
 *
 *  This function creates a new configuration entry and adds it to
 *  the configuration.
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  Tag         Tag value of new entry
 *  @param[in]  Item        Item value of new entry
 *  @param[in]  Mode        Mode flags for how to add entry
 *  @param[in]  Size        Size of entry pointed to by @c pData
 *  @param[in]  pData       Pointer to entry data
 *  @param[in,out] phCfgEntry  Pointer to storage for the handle to the
 *                          new configuration entry.
 *
 *  @remark     @c phCfgEntry is an optional pointer that can return
 *              a handle to the newly added configuration entry. When
 *              the phCfgEntry parameter is non-NULL, the function
 *              writes the referenced handle of the new configuration
 *              entry to the location specified by this parameter. It
 *              is then the caller's responsibility to dereference
 *              this handle when it is finished with it. When
 *              @c phCfgEntry is @c NULL, no entry handle is returned,
 *              but the function return value is still valid.
 *
 *  @remark     Configuration entry handles are dereferenced by calling
 *              one of the following:
 *                - CfgEntryDeRef(): stop using the entry
 *                - CfgRemoveEntry(): stop using entry and remove it
 *                      from the configuration
 *                - CfgGetNextEntry(): stop using the entry and get
 *                      the next entry
 *
 *  @remark     If the execution state of the configuration is active
 *              (see CfgExecute()), the addition of the configuration
 *              entry is immediately reflected in the operating state
 *              of the system.
 *
 *  @remark     On a service error, the configuration entry is still
 *              added to the configuration, and an entry handle is
 *              written to @c phCfgEntry, when supplied.
 *
 *  @remark     Multiple configuration entries can exist with the same
 *              @c Tag and @c Item key values. The system creates a
 *              third key (Instance) to track these duplicate keyed
 *              entries. However, by default, the configuration system
 *              does not allow for fully duplicate entries. Entries
 *              are full duplicates if there exists another entry with
 *              the same @c Tag and @c Item key values and an exact
 *              duplicate data section (size and content). When a full
 *              duplicate entry is detected, the new (duplicate) entry
 *              is not created.
 *
 *  @remark     There are some options that determine how the entry is
 *              added to the configuration by using flags that can be
 *              set in the @c Mode parameter. The default behavior when
 *              adding an object is as follows:
 *                 - Multiple instances with the same @c Tag and @c Item
 *                   values are allowed.
 *                 - However, duplicate instances with the same @c Tag,
 *                   @c Item, @c Size, and @c pData contents are ignored.
 *                 - New entries are saved to the linear buffer if or
 *                   when CfgSave() is used.
 *
 *  @remark     To modify the default behavior, one or more of the
 *              following flags can be set:
 *                 - #CFG_ADDMODE_UNIQUE: Replace all previous entry
 *                   instances with this single entry.
 *                 - #CFG_ADDMODE_DUPLICATE: Allow full duplicate entry
 *                   (duplicate @c Tag, @c Item, and entry data).
 *                   Requests to add duplicates are normally ignored.
 *                 - #CFG_ADDMODE_NOSAVE: Do not include this entry in
 *                   the linear buffer in CfgSave().
 *
 *  @remark     Setting both the #CFG_ADDMODE_UNIQUE and
 *              #CFG_ADDMODE_DUPLICATE flags is the same as only
 *              setting #CFG_ADDMODE_UNIQUE.
 *
 *  @return     Success:
 *                 - 1 on success with successful processing by a
 *                     service callback function (see CfgSetService())
 *                 - 0 on success with no processing performed by a
 *                     service callback function
 *
 *  @return     Failure:
 *                  - < 0 but > #CFGERROR_SERVICE on a configuration
 *                     error.  Possible configuration errors are:
 *                        * #CFGERROR_BADHANDLE
 *                        * #CFGERROR_BADPARAM
 *                        * #CFGERROR_RESOURCES
 *                  - <= #CFGERROR_SERVICE when the service callback
 *                     function returns an error. Service errors are
 *                     specific to the service callback functions
 *                     installed and are thus implementation dependent.
 *                     The original error return from the service
 *                     callback can be retrieved by using
 *                     CFG_GET_SERVICE_ERROR().
 *
 *  @sa CfgRemoveEntry()
 *  @sa CfgEntryDeRef()
 *  @sa CfgRemoveEntry()
 *  @sa CfgGetNextEntry()
 *  @sa CFG_GET_SERVICE_ERROR()
 */
extern int  CfgAddEntry( void *hCfg, uint32_t Tag, uint32_t Item,
                         uint32_t Mode, int Size, unsigned char *pData,
                         void **phCfgEntry );

/* Add Entry Flags */
#define CFG_ADDMODE_UNIQUE      0x0001  /**< Replace all previous instances */
#define CFG_ADDMODE_DUPLICATE   0x0002  /**< Allow duplicate data entry */
#define CFG_ADDMODE_NOSAVE      0x0004  /**< Don't include this entry in CfgSave() */

/*!
 *  @brief      Remove config entry from configuration by handle
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  hCfgEntry   Configuration entry to remove
 *
 *  @remark     This function removes a config entry from a configuration.
 *
 *  @remark     If the execution state of the configuration is active (see
 *              CfgExecute()), then the removal of the configuration
 *              entry is immediately reflected in the operating state
 *              of the system.
 *
 *  @remark     This function also performs a single dereference operation
 *              on the configuration entry handle, so the handle is
 *              invalid after the call (unless there was more than one
 *              reference made). Although the entry handle is not
 *              freed until all handle references have been removed,
 *              it is always removed from the configuration
 *              immediately.
 *
 *  @note       @c hCfgEntry is not dereferenced in the event of an error.
 *
 *  @return     Success: 0
 *  @return     Failure: < 0, @cfgerrorcodes
 *
 *  @sa CfgExecute()
 */
extern int  CfgRemoveEntry( void *hCfg, void *hCfgEntry );

/*!
 *  @brief      Get the number of entry instances for a Tag/Item pair
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  Tag         Tag value of entry
 *  @param[in]  Item        Item value of entry
 *
 *  @remark     This function searches the configuration for all instances
 *              matching the supplied @c Tag and @c Item parameters
 *              and returns the number of instances found.
 *
 *  @return     Success: >= 0 - the number of instances found
 *  @return     Failure: < 0, @cfgerrorcodes
 *
 *  @sa CfgGetEntry()
 */
extern int  CfgGetEntryCnt( void *hCfg, uint32_t Tag, uint32_t Item );

/*!
 *  @brief      Get configuration entry from configuration
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  Tag         Tag value of entry
 *  @param[in]  Item        Item value of entry
 *  @param[in]  Index       Instance index to get (1 to n)
 *  @param[in,out] phCfgEntry Optional pointer to location to store
 *                           config entry handle
 *
 *  @remark     This function searches the configuration for an entry
 *              matching the supplied @c Tag and @c Item parameters
 *              and an index matching the supplied @c Index
 *              parameter. For example, when Index is 1, the first
 *              instance is returned, when Index is 2, the second
 *              instance is returned. The total number of instances
 *              can be found by calling CfgGetEntryCnt().
 *
 *  @remark     The @c phCfgEntry parameter is an optional pointer that
 *              can return the handle of the configuration entry
 *              found. When @c phCfgEntry parameter is valid, the
 *              referenced handle of the configuration entry found is
 *              written into to this pointer. It is the caller's
 *              responsibility to dereference the handle when it is no
 *              longer needed. When the parameter @c phCfgEntry is
 *              NULL, no entry handle is returned, but the function
 *              return value is still valid (found or not found).
 *
 *  @remark     Configuration entry handles are dereferenced by
 *              calling one of the following:
 *                 * CfgEntryDeRef(): Stop using the entry
 *                 * CfgRemoveEntry(): Stop using entry and remove it
 *                   from the configuration
 *                 * CfgGetNextEntry(): Stop using entry and get next entry
 *
 *  @note        Do not attempt to use the @c Index value to enumerate all
 *               entry instances in the configuration. The index of an
 *               entry handle is valid only at the time of the call as
 *               an item can move up and down in the list as
 *               configuration changes are made. To enumerate every
 *               entry for a @c Tag / @c Item pair, start with @c
 *               Index 1, and then use CfgGetNextEntry() to get
 *               additional entries.
 *
 *  @return     Success: 1 - a matching entry was found
 *  @return     Failure:
 *                * 0 - a matching entry was not found
 *                * < 0, @cfgerrorcodes
 *
 *  @sa CfgEntryDeRef()
 *  @sa CfgRemoveEntry()
 *  @sa CfgGetNextEntry()
 */
extern int  CfgGetEntry( void *hCfg, uint32_t Tag, uint32_t Item,
                         uint32_t Index, void **phCfgEntry );

/*!
 *  @brief      Get the next entry instance matching the supplied entry handle
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  hCfgEntry   Handle to last configuration entry
 *  @param[in,out] phCfgEntryNext Optional pointer to receive handle
 *                           to next config entry
 *
 *  @remark     This function serves two purposes. First, it dereferences
 *              the configuration entry handle supplied in
 *              @c hCfgEntry. After this call, the handle is invalid
 *              (unless there was more than one reference to
 *              it). Secondly, this function returns a referenced
 *              configuration entry handle to the next instance (if
 *              any) of an entry that matches the Tag and Item values
 *              of the supplied entry.
 *
 *  @remark     When the parameter @c phCfgEntryNext is NULL, no entry
 *              handle is returned, but the function always returns 1
 *              if such an entry was found and 0 when not.
 *
 *  @remark     When @c phCfgEntryNext is not NULL, the function writes a
 *              referenced handle to the configuration entry to the
 *              location specified by this parameter. It is then the
 *              caller's responsibility to dereference this handle
 *              when it is finished with it.
 *
 *  @remark     Configuration entry handles are dereferenced by
 *              calling one of the following:
 *                 * CfgEntryDeRef(): Stop using the entry
 *                 * CfgRemoveEntry(): Stop using entry and remove it
 *                   from the configuration
 *                 * CfgGetNextEntry(): Stop using entry and get next entry
 *
 *  @note       @c hCfgEntry is not dereferenced in the event of an error.
 *
 *  @return     Success: 1 - a next entry was found
 *  @return     Failure:
 *                * 0 - a next entry was not found
 *                * < 0, @cfgerrorcodes
 *
 *  @sa CfgGetEntry()
 */
extern int  CfgGetNextEntry( void *hCfg, void *hCfgEntry,
                             void **phCfgEntryNext );

/*!
 *  @brief      Get config entry data
 *
 *  @param[in]  hCfg        Handle to configuration
 *  @param[in]  Tag         Tag value of entry
 *  @param[in]  Item        Item value of entry
 *  @param[in]  Index       Instance index to get (1 to n)
 *  @param[in]  Size        Size of buffer to receive data
 *  @param[in,out] pData    Pointer to data buffer to receive data
 *
 *  @remark     This function is a useful shortcut when searching the
 *              configuration for well-known entries. It searches the
 *              configuration for entries matching the supplied @c Tag
 *              and @c Item parameters and uses the item matching the
 *              supplied @c Index parameter. For example, if @c Index
 *              is 1, the first instance is used, if @c Index is 2,
 *              the second instance is used. The total number of
 *              instances can be found by calling CfgGetEntryCnt().
 *
 *  @remark     Instead of returning a referenced handle to the
 *              configuration entry (as with the more generic
 *              CfgGetEntry() function), this function immediately
 *              gets the entry data for this entry and copies it to
 *              the data buffer pointed to by @c pData.
 *
 *  @remark     The increased simplicity does decrease the function's
 *              flexibility. This function returns the number of bytes
 *              copied, so it will return 0 for any of the following
 *              reasons:
 *                 * A supplied parameter is incorrect
 *                 * The item was not found
 *                 * The supplied buffer size (specified by @c Size) was
 *                   not large enough to hold the data
 *
 *  @return     The number of bytes copied
 *
 *  @sa CfgGetEntry()
 */
extern int CfgGetImmediate( void *hCfg, uint32_t Tag, uint32_t Item,
                            uint32_t Index, int Size, unsigned char *pData );

/*!
 *  @brief      Add a reference to a configuration entry handle
 *
 *  @param[in]  hCfgEntry   Handle to configuration entry
 *
 *  @remark     This function adds a reference to the configuration entry
 *              handle supplied in @c hCfgEntry. It is called by an
 *              application when it intends to use a configuration
 *              entry handle beyond the scope of the function that
 *              obtained it from the configuration. This normally
 *              occurs when one user function calls another and passes
 *              it a handle.
 *
 *  @remark     Configuration entry handles should be dereferenced when no
 *              longer needed by calling one of the following:
 *                - CfgEntryDeRef(): stop using the entry
 *                - CfgRemoveEntry(): stop using entry and remove it
 *                      from the configuration
 *                - CfgGetNextEntry(): stop using the entry and get
 *                      the next entry
 *
 *  @return     Success: 0
 *  @return     Failure: < 0, @cfgerrorcodes
 *
 *  @sa CfgEntryDeRef()
 *  @sa CfgRemoveEntry()
 *  @sa CfgGetNextEntry()
 */
extern int  CfgEntryRef( void *hCfgEntry );

/*!
 *  @brief      Remove a reference to a config entry handle
 *
 *  @param[in]  hCfgEntry   Handle to configuration entry
 *
 *  @remark     This function removes a reference to the configuration
 *              entry handle supplied in @c hCfgEntry. It is called by
 *              an application when it wishes to discard a referenced
 *              configuration entry handle. Once this function is
 *              called, the handle should no longer be used.

 *  @return     Success: 0
 *  @return     Failure: < 0, @cfgerrorcodes
 */
extern int  CfgEntryDeRef( void *hCfgEntry );

/*!
 *  @brief      Get configuration entry data
 *
 *  @param[in]  hCfgEntry   Handle to configuration entry
 *  @param[in,out] pSize    Pointer to size of data buffer in @c pData
 *  @param[in,out] pData    Pointer to data buffer
 *
 *  @remark     This function acquires the entry data of the configuration
 *              entry specified by the entry handle in @c hCfgEntry.
 *
 *  @remark     The value pointed to by @c pSize is set to the size of the
 *              supplied buffer, or zero to get the required size (the
 *              pointer @c pSize must be valid, but the value at the
 *              pointer can be zero). If the value at @c pSize is
 *              zero, or less than the number of bytes required to
 *              hold the entry data, this function returns 0, and the
 *              number of bytes required to hold the data is stored at
 *              @c pSize.
 *
 *  @remark     @c pData points to the data buffer to receive the
 *              configuration entry data. This pointer can be NULL if
 *              @c *pSize is zero.

 *  @return     Success: number of bytes written
 *  @return     Failure:
 *                * 0 on a size error (@c pSize will be set to required size)
 *                * < 0, @cfgerrorcodes
 */
extern int  CfgEntryGetData( void *hCfgEntry, int *pSize, unsigned char *pData );

/*!
 *  @brief      Set/reset configuration entry data
 *
 *  @param[in]  hCfgEntry   Handle to configuration entry
 *  @param[in]  Size        Size of data buffer in @c pData
 *  @param[in]  pData       Pointer to data buffer
 *
 *  @remark     This function replaces the entry data of the configuration
 *              entry specified by @c hCfgEntry.
 *
 *  @remark     The new entry data is pointed to by @c pData, with a
 *              size indicated by @c Size. Note that the new data must
 *              be an exact replacement for the old. The size of the
 *              new buffer must exactly match the old size.
 *
 *  @remark     This function should be used for configuration entries
 *              that are for information purposes only. Note that if a
 *              service provider callback is associated with the Tag
 *              value of this entry, the processing function is not
 *              called as a result of this data update. This function
 *              only updates the data stored for this configuration
 *              entry.
*
 *  @return     Success: number of bytes written
 *  @return     Failure:
 *                * 0 on a size error (new size does not match old size)
 *                * < 0, @cfgerrorcodes
 *
 *  @sa CfgEntryGetData
 */
extern int  CfgEntrySetData( void *hCfgEntry, int Size, unsigned char *pData );

/*!
 *  @brief      Get information on a configuration entry
 *
 *  @param[in]  hCfgEntry   Handle to configuration entry
 *  @param[in,out] pSize    Pointer to receive the size of the config entry
 *                          data buffer
 *  @param[in,out] ppData   Location to receive the pointer to the config*                                   entry data buffer
 *
 *  @remark     This function acquires the size and pointer to a
 *               configuration entry's data buffer.
 *
 *  @remark     The entry handle is supplied @c hCfgEntry. A pointer to
 *              receive the size of the entry's data buffer is
 *              supplied in @c pSize, and a pointer to receive a pointer
 *              to the entry's data buffer is supplied in
 *              @c ppData. Either pointer parameter can be left NULL if
 *              the information is not required.
 *
 *  @remark     This function should be used with great care. Direct
 *              manipulation of the configuration entry data should
 *              only be attempted on informational tags, and only when
 *              the caller holds a referenced handle to the
 *              configuration entry. This function is used in
 *              configuration service callback functions, which are
 *              called only when the configuration is in a protected
 *              state.
*
 *  @return     Success: 0
 *  @return     Failure: < 0, @cfgerrorcodes
 */
extern int  CfgEntryInfo( void *hCfgEntry, int *pSize, unsigned char **ppData );

/*! @} */

/*!
 *  @brief      Config API error-related services
 *
 *  @addtogroup ti_ndk_inc_nettools_inc__Cfg_CFGERROR Configuration Manager Error Codes
 *  @ingroup ti_ndk_inc_nettools_inc__Cfg
 *  @{
 */
#define CFGERROR_BADHANDLE      -1      /**< Invalid Cfg handle */
#define CFGERROR_BADPARAM       -2      /**< Invalid function parameter */
#define CFGERROR_RESOURCES      -3      /**< Memory allocation error */
#define CFGERROR_REFERROR       -4      /**< Reference count mismatch */
#define CFGERROR_ALREADY        -5      /**< Already in desired state */
#define CFGERROR_SERVICE        -100    /**< First service error */

#define CFG_MAKE_CFGERROR(x)     ((x)+CFGERROR_SERVICE)

/*!
 *  @brief      Get the original service error
 *
 *  @param[in]  x       CfgAddEntry() return value
 *
 *  @code
 *      errorCode = CFG_GET_SERVICE_ERROR(CfgAddEntryRetVal);
 *  @endcode
 *  @sa CfgAddEntry()
 */
#define CFG_GET_SERVICE_ERROR(x) ((x)-CFGERROR_SERVICE)
#define CFG_IS_SERVICE_ERROR(x)  ((x)<=CFGERROR_SERVICE)

/*! @} */
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
