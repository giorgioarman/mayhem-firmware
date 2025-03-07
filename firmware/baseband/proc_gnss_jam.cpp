/*
 * Copyright (C) 2016 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
 * Copyright (C) 2020 Shao
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "proc_gnss_jam.hpp"
#include "sine_table_int8.hpp"
#include "portapack_shared_memory.hpp"

#include "event_m4.hpp"

#include "utility.hpp"

GNSSJamProcessor::GNSSJamProcessor() {
    configured = false;
    baseband_thread.start();
}

void GNSSJamProcessor::execute(const buffer_c8_t& buffer) {
    /* 20MHz, 2048 samples */

    if (!configured || !stream) return;

    // File data is in C8 format, and 20MHz
    // To fill up the 2048-sample C8 buffer @ 2 bytes per sample = 4096 bytes
    const size_t bytes_to_read = sizeof(*buffer.p) * 1 * (buffer.count);
    size_t bytes_read_this_iteration = stream->read(iq_buffer.p, bytes_to_read);
    size_t samples_read_this_iteration = bytes_read_this_iteration / sizeof(*buffer.p);

    bytes_read += bytes_read_this_iteration;

    // NB: Couldn't we have just read the data into buffer.p to start with, or some DMA/cache coherency concern?
    //
    // for (size_t i = 0; i < buffer.count; i++) {
    //     auto re_out = iq_buffer.p[i].real();
    //     auto im_out = iq_buffer.p[i].imag();
    //     buffer.p[i] = {(int8_t)re_out, (int8_t)im_out};
    // }
    memcpy(buffer.p, iq_buffer.p, bytes_read_this_iteration);  // memcpy should be more efficient than 1 byte at a time
}

void GNSSJamProcessor::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::ReplayConfig:
            configured = false;
            bytes_read = 0;
            replay_config(*reinterpret_cast<const ReplayConfigMessage*>(message));
            break;

        // App has prefilled the buffers, we're ready to go now
        case Message::ID::FIFOData:
            configured = true;
            break;

        default:
            break;
    }
}

void GNSSJamProcessor::replay_config(const ReplayConfigMessage& message) {
    if (message.config) {
        stream = std::make_unique<StreamOutput>(message.config);

        // Tell application that the buffers and FIFO pointers are ready, prefill
        shared_memory.application_queue.push(sig_message);
    } else {
        stream.reset();
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<GNSSJamProcessor>()};
    event_dispatcher.run();
    return 0;
}
