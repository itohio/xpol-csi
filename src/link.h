#pragma once
// UART transport for the inter-node link. Wraps the framing in link_proto.

#include "link_proto.h"

void link_init(LinkParser::Handler handler);

// Send a typed message to the peer node.
void link_send(MsgType type, const void* payload, uint16_t len);
