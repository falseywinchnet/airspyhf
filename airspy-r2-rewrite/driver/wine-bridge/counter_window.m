#import <Cocoa/Cocoa.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "proto.h"

enum {
    ROW_CAPTURED,
    ROW_USB_RETIRED,
    ROW_HOST_DROPS,
    ROW_ADC_FIFO,
    ROW_ADC_DESCRIPTOR,
    ROW_ADC_RANGE,
    ROW_DMA_ERRORS,
    ROW_DMA_RECOVERIES,
    ROW_DMA_RECOVERY_FAILURES,
    ROW_DMA_DROPPED,
    ROW_USB_DTD_ERRORS,
    ROW_USB_QUEUE_RECOVERIES,
    ROW_USB_PARTIAL,
    ROW_USB_BACKPRESSURE,
    ROW_BACKPRESSURE_LOSS,
    ROW_STEERING_DISCARDS,
    ROW_STEERING_RUNS,
    ROW_STEERING_MINIMUM_AVAILABLE,
    ROW_STEERING_ALTERNATION,
    ROW_STEERING_NO_CANDIDATE,
    ROW_STALE_COMPLETIONS,
    ROW_USB_CONTROLLER_ERRORS,
    ROW_USB_BUS_RESETS,
    ROW_SUSPEND_LOSS,
    ROW_OWNERSHIP_HALTS,
    ROW_OWNERSHIP_OVERWRITES,
    ROW_COUNT
};

static int write_full(int fd, const void *data, size_t length)
{
    const uint8_t *cursor = data;
    while (length) {
        ssize_t result = send(fd, cursor, length, 0);
        if (result < 0 && errno == EINTR) continue;
        if (result <= 0) return -1;
        cursor += result;
        length -= (size_t)result;
    }
    return 0;
}

static int read_full(int fd, void *data, size_t length)
{
    uint8_t *cursor = data;
    while (length) {
        ssize_t result = recv(fd, cursor, length, 0);
        if (result < 0 && errno == EINTR) continue;
        if (result <= 0) return -1;
        cursor += result;
        length -= (size_t)result;
    }
    return 0;
}

static NSString *event_name(uint32_t event)
{
    switch (event) {
    case AOB_MONITOR_HELPER_STARTED: return @"helper started";
    case AOB_MONITOR_DEVICE_OPENED: return @"radio opened";
    case AOB_MONITOR_STREAM_STARTED: return @"stream started";
    case AOB_MONITOR_STREAM_STOPPED: return @"stream stopped";
    case AOB_MONITOR_SAMPLERATE_CHANGED: return @"sample rate changed";
    case AOB_MONITOR_DEVICE_CLOSED: return @"radio closed";
    default: return @"waiting";
    }
}

static NSString *mode_name(uint32_t mode)
{
    switch (mode) {
    case 0: return @"legacy/off";
    case 1: return @"synthetic";
    case 2: return @"ADC ring";
    case 3: return @"recovering";
    default: return @"unknown";
    }
}

@interface AirspyCounterController : NSObject <NSApplicationDelegate,
                                               NSWindowDelegate>
@end

@implementation AirspyCounterController {
    NSWindow *_window;
    NSTextField *_status;
    NSTextField *_session;
    NSMutableArray<NSTextField *> *_values;
    int _socket;
    BOOL _haveBaseline;
    aob_monitor_snapshot _baseline;
}

- (NSTextField *)labelWithText:(NSString *)text
                         frame:(NSRect)frame
                          bold:(BOOL)bold
{
    NSTextField *field = [[NSTextField alloc] initWithFrame:frame];
    field.stringValue = text;
    field.editable = NO;
    field.selectable = NO;
    field.bezeled = NO;
    field.drawsBackground = NO;
    field.font = bold ? [NSFont boldSystemFontOfSize:13]
                      : [NSFont systemFontOfSize:12];
    return field;
}

- (void)applicationDidFinishLaunching:(NSNotification *)notification
{
    (void)notification;
    _socket = -1;
    _values = [NSMutableArray arrayWithCapacity:ROW_COUNT];

    _window = [[NSWindow alloc]
        initWithContentRect:NSMakeRect(100, 100, 480, 756)
                  styleMask:NSWindowStyleMaskTitled |
                            NSWindowStyleMaskClosable |
                            NSWindowStyleMaskMiniaturizable
                    backing:NSBackingStoreBuffered
                      defer:NO];
    _window.title = @"Airspy Stream Counters";
    _window.delegate = self;
    _window.level = NSFloatingWindowLevel;
    [_window setFrameAutosaveName:@"AirspyStreamCountersWindow"];

    NSView *content = _window.contentView;
    _status = [self labelWithText:@"Connecting to helper…"
                            frame:NSMakeRect(18, 714, 444, 22) bold:YES];
    _session = [self labelWithText:@""
                             frame:NSMakeRect(18, 692, 444, 20) bold:NO];
    [content addSubview:_status];
    [content addSubview:_session];

    NSArray<NSString *> *names = @[
        @"Captured 16 KiB banks",
        @"USB-retired banks",
        @"Host-driver dropped samples",
        @"ADC FIFO overflows",
        @"ADC descriptor faults",
        @"ADC over / under-range",
        @"GPDMA errors",
        @"GPDMA recoveries",
        @"GPDMA recovery failures",
        @"Estimated dropped banks",
        @"USB dTD errors",
        @"USB queue recoveries",
        @"USB partial transfers",
        @"USB queue backpressure",
        @"Backpressure sample-loss events",
        @"Deliberately discarded banks",
        @"Consecutive discard runs",
        @"Minimum reusable banks",
        @"SRAM alternation violations",
        @"No-candidate steering faults",
        @"Stale-generation completions",
        @"USB controller error IRQs",
        @"USB bus resets",
        @"Suspend discontinuities",
        @"Ownership-protection halts",
        @"Ownership overwrites"
    ];

    CGFloat y = 662;
    for (NSUInteger index = 0; index < names.count; ++index, y -= 24) {
        NSTextField *name = [self labelWithText:names[index]
                                          frame:NSMakeRect(24, y, 325, 20)
                                           bold:NO];
        NSTextField *value = [self labelWithText:@"0"
                                           frame:NSMakeRect(354, y, 100, 20)
                                            bold:YES];
        value.alignment = NSTextAlignmentRight;
        [content addSubview:name];
        [content addSubview:value];
        [_values addObject:value];
    }

    NSTextField *note = [self
        labelWithText:@"View resets on start, stop, or sample-rate change."
                 frame:NSMakeRect(18, 18, 444, 20) bold:NO];
    note.textColor = NSColor.secondaryLabelColor;
    [content addSubview:note];

    [_window makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];

    [NSTimer scheduledTimerWithTimeInterval:1.0
                                    target:self
                                  selector:@selector(poll:)
                                  userInfo:nil
                                   repeats:YES];
    [self poll:nil];
}

- (BOOL)windowShouldClose:(NSWindow *)sender
{
    (void)sender;
    [NSApp terminate:nil];
    return YES;
}

- (void)disconnect
{
    if (_socket >= 0) close(_socket);
    _socket = -1;
}

- (BOOL)connectHelper
{
    if (_socket >= 0) return YES;

    int port = AOB_DEFAULT_PORT;
    const char *configured = getenv(AOB_PORT_ENV);
    if (configured && atoi(configured) > 0) port = atoi(configured);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return NO;
    struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_port = htons((uint16_t)port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        close(fd);
        return NO;
    }
    uint32_t channel = AOB_CHANNEL_CONTROL;
    if (write_full(fd, &channel, sizeof(channel)) != 0) {
        close(fd);
        return NO;
    }
    _socket = fd;
    return YES;
}

- (BOOL)readSnapshot:(aob_monitor_snapshot *)snapshot
{
    if (![self connectHelper]) return NO;
    aob_req_hdr request = {
        .op = AOB_OP_GET_MONITOR_SNAPSHOT,
        .dev = 0,
        .in_len = 0
    };
    aob_resp_hdr response;
    if (write_full(_socket, &request, sizeof(request)) != 0 ||
        read_full(_socket, &response, sizeof(response)) != 0 ||
        response.ret != 0 ||
        response.out_len != sizeof(*snapshot) ||
        read_full(_socket, snapshot, sizeof(*snapshot)) != 0) {
        [self disconnect];
        return NO;
    }
    return snapshot->magic == AOB_MONITOR_MAGIC &&
           snapshot->version == AOB_MONITOR_VERSION;
}

- (void)setRow:(NSUInteger)row
         value:(uint64_t)value
         fault:(BOOL)fault
{
    NSTextField *field = _values[row];
    field.stringValue =
        [NSNumberFormatter localizedStringFromNumber:@(value)
                                          numberStyle:NSNumberFormatterDecimalStyle];
    field.textColor = fault && value ? NSColor.systemRedColor
                                    : NSColor.labelColor;
}

- (void)setRangeRow:(uint32_t)over under:(uint32_t)under
{
    NSTextField *field = _values[ROW_ADC_RANGE];
    field.stringValue = [NSString stringWithFormat:@"%u / %u", over, under];
    field.textColor = (over || under) ? NSColor.systemOrangeColor
                                     : NSColor.labelColor;
}

- (void)clearRows
{
    for (NSTextField *field in _values) {
        field.stringValue = @"0";
        field.textColor = NSColor.labelColor;
    }
}

- (void)poll:(NSTimer *)timer
{
    (void)timer;
    aob_monitor_snapshot snapshot;
    if (![self readSnapshot:&snapshot]) {
        _status.stringValue = @"Helper unavailable — reconnecting…";
        _status.textColor = NSColor.systemOrangeColor;
        _session.stringValue = @"";
        return;
    }

    if (!snapshot.telemetry_valid) {
        _status.stringValue = @"Helper connected — waiting for the radio";
        _status.textColor = NSColor.secondaryLabelColor;
        _session.stringValue =
            [NSString stringWithFormat:@"%@ • telemetry result %d",
             event_name(snapshot.event), snapshot.telemetry_result];
        _haveBaseline = NO;
        [self clearRows];
        return;
    }

    BOOL reset = !_haveBaseline ||
                 snapshot.session_epoch != _baseline.session_epoch;
    if (reset) {
        _baseline = snapshot;
        _haveBaseline = YES;
    }

    const aob_stream_telemetry *c = &snapshot.telemetry;
    const aob_stream_telemetry *b = &_baseline.telemetry;
#define DELTA(field) ((uint32_t)(c->field - b->field))

    _status.stringValue = snapshot.streaming
        ? [NSString stringWithFormat:@"Running • %@ • session %llu",
           mode_name(c->mode), snapshot.session_epoch]
        : [NSString stringWithFormat:@"Stopped • %@ • session %llu",
           mode_name(c->mode), snapshot.session_epoch];
    _status.textColor = snapshot.streaming ? NSColor.systemGreenColor
                                           : NSColor.labelColor;
    NSString *rate = snapshot.sample_rate
        ? [NSString stringWithFormat:@"%.3f MS/s requested real rate • ",
           snapshot.sample_rate / 1000000.0]
        : @"";
    _session.stringValue =
        [NSString stringWithFormat:@"%@%@%@",
         rate, event_name(snapshot.event),
         reset ? @" • counters reset" : @""];

    [self setRow:ROW_CAPTURED value:DELTA(capture_completed) fault:NO];
    [self setRow:ROW_USB_RETIRED value:DELTA(usb_retired) fault:NO];
    [self setRow:ROW_HOST_DROPS
           value:snapshot.host_dropped_samples -
                 _baseline.host_dropped_samples fault:YES];
    [self setRow:ROW_ADC_FIFO value:DELTA(adc_fifo_overflow_count) fault:YES];
    [self setRow:ROW_ADC_DESCRIPTOR
           value:DELTA(adc_descriptor_error_count) fault:YES];
    [self setRangeRow:DELTA(adc_overrange_count)
                under:DELTA(adc_underrange_count)];
    [self setRow:ROW_DMA_ERRORS value:DELTA(dma_error_count) fault:YES];
    [self setRow:ROW_DMA_RECOVERIES value:DELTA(dma_recovery_count) fault:YES];
    [self setRow:ROW_DMA_RECOVERY_FAILURES
           value:DELTA(dma_recovery_failure_count) fault:YES];
    [self setRow:ROW_DMA_DROPPED
           value:DELTA(dma_recovery_dropped_buffer_estimate) fault:YES];
    [self setRow:ROW_USB_DTD_ERRORS value:DELTA(usb_errors) fault:YES];
    [self setRow:ROW_USB_QUEUE_RECOVERIES
           value:DELTA(usb_queue_recovery_count) fault:YES];
    [self setRow:ROW_USB_PARTIAL value:DELTA(usb_partial) fault:YES];
    [self setRow:ROW_USB_BACKPRESSURE
           value:DELTA(usb_backpressure) fault:YES];
    [self setRow:ROW_BACKPRESSURE_LOSS
           value:DELTA(backpressure_discontinuity_count) fault:YES];
    [self setRow:ROW_STEERING_DISCARDS
           value:DELTA(steering_overwrites) fault:NO];
    [self setRow:ROW_STEERING_RUNS
           value:DELTA(steering_overwrite_runs) fault:NO];
    [self setRow:ROW_STEERING_MINIMUM_AVAILABLE
           value:c->steering_minimum_available fault:NO];
    [self setRow:ROW_STEERING_ALTERNATION
           value:DELTA(steering_alternation_violations) fault:YES];
    [self setRow:ROW_STEERING_NO_CANDIDATE
           value:DELTA(steering_no_candidate_faults) fault:YES];
    [self setRow:ROW_STALE_COMPLETIONS
           value:DELTA(stale_generation_completions) fault:YES];
    [self setRow:ROW_USB_CONTROLLER_ERRORS
           value:DELTA(usb_controller_error_irq_count) fault:YES];
    [self setRow:ROW_USB_BUS_RESETS
           value:DELTA(usb_bus_reset_count) fault:YES];
    [self setRow:ROW_SUSPEND_LOSS
           value:DELTA(usb_suspend_discontinuities) fault:YES];
    [self setRow:ROW_OWNERSHIP_HALTS
           value:DELTA(overwrite_prevented) fault:YES];
    [self setRow:ROW_OWNERSHIP_OVERWRITES
           value:DELTA(ownership_overwrite_count) fault:YES];
#undef DELTA
}

@end

int main(void)
{
    @autoreleasepool {
        NSApplication *application = [NSApplication sharedApplication];
        application.activationPolicy = NSApplicationActivationPolicyRegular;
        AirspyCounterController *controller =
            [[AirspyCounterController alloc] init];
        application.delegate = controller;
        [application run];
    }
    return 0;
}
