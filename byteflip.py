import re

START_TAG = 'byteflip-begin'
END_TAG = 'byteflip-end'


def flipHexByte(byteHexMatch):
    byteHexStr = byteHexMatch.group()

    inB = int(byteHexStr, 16)
    outB = 0

    inMask = 1 << 7
    outMask = 1
    while inMask != 0:
        if (inB & inMask) != 0:
            outB |= outMask

        inMask = inMask >> 1
        outMask = outMask << 1

    return '%#04x' % outB


def flipAllBytesInLine(line):
    return re.sub(r'0x[0-9a-fA-F][0-9a-fA-F]', flipHexByte, line)


def flipBytes(inFile, outFile):
    active = False

    for line in inFile.readlines():
        if active:
            if END_TAG in line:
                active = False
            else:
                outFile.write(flipAllBytesInLine(line))
        
        if not active:
            outFile.write(line)

            if START_TAG in line:
                active = True



with open('eyeSquare.ino') as inFile:
    with open('flipped.ino', 'wt') as outFile:
        flipBytes(inFile, outFile)