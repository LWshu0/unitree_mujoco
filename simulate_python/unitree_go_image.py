from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Image_(idl.IdlStruct, typename="unitree_go.msg.dds_.Image_"):
    timestamp: types.uint64
    width: types.uint32
    height: types.uint32
    step: types.uint32
    encoding: str
    data: types.sequence[types.uint8]


def unitree_go_msg_dds__Image_():
    return Image_(0, 0, 0, 0, "", [])
