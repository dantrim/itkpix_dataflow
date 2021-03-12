#!/bin/env/python

from argparse import ArgumentParser


def main() :

    parser = ArgumentParser()
    parser.add_argument("input", type = str, help = "Input 64-bit binary block")
    args = parser.parse_args()

    block = args.input.strip()
    data = list(block)

    pos = 0

    # NS bit
    w = 1
    ns_bit = "".join(data[pos:pos+w])
    pos += 1
    
    # CH id
    w = 2
    ch_id = "".join(data[pos:pos+w])
    pos += w

    # tag
    w = 8
    tag = "".join(data[pos:pos+w])
    pos += w

    # ccol
    w = 6
    ccol = "".join(data[pos:pos+w])
    pos += w

    # is_last
    w = 1
    is_last = "".join(data[pos:pos+w])
    pos += w

    # is_neighbor
    w = 1
    is_neighbor = "".join(data[pos:pos+w])
    pos += w

    # qrow
    w = 8
    qrow = "".join(data[pos:pos+w])
    pos += w

    # hitmap (assumed to be uncompressed)
    w = 16
    hitmap = "".join(data[pos:pos+w])
    pos += w


    print(55 * '-')
    print(f"NS          {ns_bit:>55}")
    print(f"ch_id       {ch_id:>55}")
    print(f"tag         {tag:>55}")
    print(f"ccol        {ccol:>55}")
    print(f"is_last     {is_last:>55}")
    print(f"is_neighbor {is_neighbor:>55}")
    print(f"qrow        {qrow:>55}")
    print(f"hitmap      {hitmap:>55}")



if __name__ == "__main__" :
    main()
