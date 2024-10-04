#include "deployment.h"

const struct id_addr deployment_id_addr_list[] = {
{160, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x63, 0x82, 0x36}}, //DEPT
{161, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x74, 0x87, 0x22}}, //DEPT
{162, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x75, 0x0a, 0x1f}}, //DEPT
{163, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x60, 0x4a, 0xba}}, //DEPT
{164, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x60, 0x06, 0xb9}}, //DEPT
{165, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x52, 0xda, 0x21}}, //DEPT
{166, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x74, 0xc4, 0x20}}, //DEPT
{167, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x74, 0xc3, 0x02}}, //DEPT
{168, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x74, 0x9d, 0x38}}, //DEPT
{169, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x74, 0x56, 0x2d}}, //DEPT
{170, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x74, 0x4d, 0x14}}, //DEPT
{171, {0x08, 0x2b, 0x31, 0x07, 0xcc, 0x74, 0x91, 0x11}}, //DEPT
{172, {0x05, 0xc3, 0x22, 0x08, 0x4a, 0xd4, 0x0f, 0xa8}}, //DEPT
{173, {0x05, 0xc3, 0x22, 0x08, 0x4a, 0xd4, 0x8c, 0x1b}}, //DEPT

{0, {0,0,0,0,0,0,0,0}}
};

const unsigned int deployment_num_nodes = sizeof(deployment_id_addr_list)/sizeof(struct id_addr);