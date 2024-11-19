#ifndef USBH_INTERNAL_H_
#error "Internal use only"
#endif

#ifndef _USBH_DEBUG_HELPER_ENABLE_TRACE
#error "_USBH_DEBUG_HELPER_ENABLE_TRACE must be set"
#endif
#ifndef _USBH_DEBUG_HELPER_ENABLE_INFO
#error "_USBH_DEBUG_HELPER_ENABLE_INFO must be set"
#endif
#ifndef _USBH_DEBUG_HELPER_ENABLE_WARNINGS
#error "_USBH_DEBUG_HELPER_ENABLE_WARNINGS must be set"
#endif
#ifndef _USBH_DEBUG_HELPER_ENABLE_ERRORS
#error "_USBH_DEBUG_HELPER_ENABLE_ERRORS must be set"
#endif

#define _usbh_dbg_host(s)  _usbh_dbg(host, s)
#define _usbh_dbg_port(s) _usbh_dbg(port->device.host, s)
#define _usbh_dbg_dev(s) _usbh_dbg(dev->host, s)
#define _usbh_dbg_ep(lvl, s) _usbh_ldbg(ep->device->host, ep->trace_level, lvl, s)
#define _usbh_dbg_urb(lvl, s) _usbh_ldbg(urb->ep->device->host, urb->ep->trace_level, lvl, s)

#define _usbh_dbgf_host(f, ...) _usbh_dbgf(host, f, ##__VA_ARGS__)
#define _usbh_dbgf_port(f, ...) _usbh_dbgf(port->device.host, f, ##__VA_ARGS__)
#define _usbh_dbgf_dev(f, ...)  _usbh_dbgf(dev->host, f, ##__VA_ARGS__)
#define _usbh_dbgf_ep(f, lvl, ...) _usbh_ldbgf(ep->device->host, ep->trace_level, lvl, "\t%s: " f, ep->name, ##__VA_ARGS__)
#define _usbh_dbgf_urb(f, lvl, ...) _usbh_ldbgf(urb->ep->device->host, urb->ep->trace_level, lvl, "\t%s: " f, urb->ep->name, ##__VA_ARGS__)

#if defined(_USBH_DEBUG_HELPER_CLASS_DRIVER)
#define _usbh_dbg_classdrv(drv, s) _usbh_dbg(drv->dev->host, s)
#define _usbh_dbgf_classdrv(drv, f, ...)  _usbh_dbgf(drv->dev->host, f, ##__VA_ARGS__)
#endif

#define _usbh_dbg_dummy	do {} while(0)

#if _USBH_DEBUG_HELPER_ENABLE_TRACE
#define udbg(s) _usbh_dbg_host(s)
#define uportdbg(s) _usbh_dbg_port(s)
#define udevdbg(s) _usbh_dbg_dev(s)
#define uepdbg(s) _usbh_dbg_ep(4, s)
#define uurbdbg(s) _usbh_dbg_urb(4, s)
#define udbgf(f, ...) _usbh_dbgf_host(f, ##__VA_ARGS__)
#define uportdbgf(f, ...) _usbh_dbgf_port(f, ##__VA_ARGS__)
#define udevdbgf(f, ...) _usbh_dbgf_dev(f, ##__VA_ARGS__)
#define uepdbgf(f, ...) _usbh_dbgf_ep(f, 4, ##__VA_ARGS__)
#define uurbdbgf(f, ...) _usbh_dbgf_urb(f, 4, ##__VA_ARGS__)
#else
#define udbg(s) _usbh_dbg_dummy
#define uportdbg(s) _usbh_dbg_dummy
#define udevdbg(s) _usbh_dbg_dummy
#define uurbdbg(s) _usbh_dbg_dummy
#define uepdbg(s) _usbh_dbg_dummy
#define udbgf(f, ...) _usbh_dbg_dummy
#define uportdbgf(f, ...) _usbh_dbg_dummy
#define udevdbgf(f, ...) _usbh_dbg_dummy
#define uepdbgf(f, ...) _usbh_dbg_dummy
#define uurbdbgf(f, ...) _usbh_dbg_dummy
#endif

#if _USBH_DEBUG_HELPER_ENABLE_INFO
#define uinfo(s) _usbh_dbg_host(s)
#define uportinfo(s) _usbh_dbg_port(s)
#define udevinfo(s) _usbh_dbg_dev(s)
#define uepinfo(s) _usbh_dbg_ep(3, s)
#define uurbinfo(s) _usbh_dbg_urb(3, s)
#define uinfof(f, ...) _usbh_dbgf_host(f, ##__VA_ARGS__)
#define uportinfof(f, ...) _usbh_dbgf_port(f, ##__VA_ARGS__)
#define udevinfof(f, ...) _usbh_dbgf_dev(f, ##__VA_ARGS__)
#define uepinfof(f, ...) _usbh_dbgf_ep(f, 3, ##__VA_ARGS__)
#define uurbinfof(f, ...) _usbh_dbgf_urb(f, 3, ##__VA_ARGS__)
#else
#define uinfo(s) _usbh_dbg_dummy
#define udevinfo(s) _usbh_dbg_dummy
#define uportinfo(s) _usbh_dbg_dummy
#define uepinfo(s) _usbh_dbg_dummy
#define uurbinfo(s) _usbh_dbg_dummy
#define uinfof(f, ...) _usbh_dbg_dummy
#define uportinfof(f, ...) _usbh_dbg_dummy
#define udevinfof(f, ...) _usbh_dbg_dummy
#define uepinfof(f, ...) _usbh_dbg_dummy
#define uurbinfof(f, ...) _usbh_dbg_dummy
#endif

#if _USBH_DEBUG_HELPER_ENABLE_WARNINGS
#define uwarn(s) _usbh_dbg_host(s)
#define uportwarn(s) _usbh_dbg_port(s)
#define udevwarn(s) _usbh_dbg_dev(s)
#define uepwarn(s) _usbh_dbg_ep(3, s)
#define uurbwarn(s) _usbh_dbg_urb(3, s)
#define uwarnf(f, ...) _usbh_dbgf_host(f, ##__VA_ARGS__)
#define uportwarnf(f, ...) _usbh_dbgf_port(f, ##__VA_ARGS__)
#define udevwarnf(f, ...) _usbh_dbgf_dev(f, ##__VA_ARGS__)
#define uepwarnf(f, ...) _usbh_dbgf_ep(f, 3, ##__VA_ARGS__)
#define uurbwarnf(f, ...) _usbh_dbgf_urb(f, 3, ##__VA_ARGS__)
#else
#define uwarn(s) _usbh_dbg_dummy
#define udevwarn(s) _usbh_dbg_dummy
#define uportwarn(s) _usbh_dbg_dummy
#define uepwarn(s) _usbh_dbg_dummy
#define uurbwarn(s) _usbh_dbg_dummy
#define uwarnf(f, ...) _usbh_dbg_dummy
#define uportwarnf(f, ...) _usbh_dbg_dummy
#define udevwarnf(f, ...) _usbh_dbg_dummy
#define uepwarnf(f, ...) _usbh_dbg_dummy
#define uurbwarnf(f, ...) _usbh_dbg_dummy
#endif


#if _USBH_DEBUG_HELPER_ENABLE_ERRORS
#define uerr(s) _usbh_dbg_host(s)
#define uporterr(s) _usbh_dbg_port(s)
#define udeverr(s) _usbh_dbg_dev(s)
#define ueperr(s) _usbh_dbg_ep(3, s)
#define uurberr(s) _usbh_dbg_urb(3, s)
#define uerrf(f, ...) _usbh_dbgf_host(f, ##__VA_ARGS__)
#define uporterrf(f, ...) _usbh_dbgf_port(f, ##__VA_ARGS__)
#define udeverrf(f, ...) _usbh_dbgf_dev(f, ##__VA_ARGS__)
#define ueperrf(f, ...) _usbh_dbgf_ep(f, 3, ##__VA_ARGS__)
#define uurberrf(f, ...) _usbh_dbgf_urb(f, 3, ##__VA_ARGS__)
#else
#define uerr(s) _usbh_dbg_dummy
#define udeverr(s) _usbh_dbg_dummy
#define uporterr(s) _usbh_dbg_dummy
#define ueperr(s) _usbh_dbg_dummy
#define uurberr(s) _usbh_dbg_dummy
#define uerrf(f, ...) _usbh_dbg_dummy
#define uporterrf(f, ...) _usbh_dbg_dummy
#define udeverrf(f, ...) _usbh_dbg_dummy
#define ueperrf(f, ...) _usbh_dbg_dummy
#define uurberrf(f, ...) _usbh_dbg_dummy
#endif

#if defined(_USBH_DEBUG_HELPER_CLASS_DRIVER)
#if _USBH_DEBUG_HELPER_ENABLE_TRACE
#define uclassdrvdbg(s) _usbh_dbg_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, s)
#define uclassdrvdbgf(f, ...) _usbh_dbgf_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, f, ##__VA_ARGS__)
#else
#define uclassdrvdbg(s) _usbh_dbg_dummy
#define uclassdrvdbgf(f, ...) _usbh_dbg_dummy
#endif
#if _USBH_DEBUG_HELPER_ENABLE_INFO
#define uclassdrvinfo(s) _usbh_dbg_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, s)
#define uclassdrvinfof(f, ...) _usbh_dbgf_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, f, ##__VA_ARGS__)
#else
#define uclassdrvinfo(s) _usbh_dbg_dummy
#define uclassdrvinfof(f, ...) _usbh_dbg_dummy
#endif
#if _USBH_DEBUG_HELPER_ENABLE_WARNINGS
#define uclassdrvwarn(s) _usbh_dbg_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, s)
#define uclassdrvwarnf(f, ...) _usbh_dbgf_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, f, ##__VA_ARGS__)
#else
#define uclassdrvwarn(s) _usbh_dbg_dummy
#define uclassdrvwarnf(f, ...) _usbh_dbg_dummy
#endif
#if _USBH_DEBUG_HELPER_ENABLE_ERRORS
#define uclassdrverr(s) _usbh_dbg_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, s)
#define uclassdrverrf(f, ...) _usbh_dbgf_classdrv(_USBH_DEBUG_HELPER_CLASS_DRIVER, f, ##__VA_ARGS__)
#else
#define uclassdrverr(s) _usbh_dbg_dummy
#define uclassdrverrf(f, ...) _usbh_dbg_dummy
#endif
#endif
