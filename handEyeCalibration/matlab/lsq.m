% least squares solution 
A = [rotRB2' * rotRB1;
    rotRB3' * rotRB2;
    rotRB4' * rotRB3;
    rotRB5' * rotRB4;
    rotRB6' * rotRB5;
    rotRB7' * rotRB6;
    rotRB8' * rotRB7;
    rotRB9' * rotRB8;
    rotRB10' * rotRB9;
    rotRB1' * rotRB10];

B = [rotCBin2' * rotCBin1;
    rotCBin3' * rotCBin2;
    rotCBin4' * rotCBin3;
    rotCBin5' * rotCBin4;
    rotCBin6' * rotCBin5;
    rotCBin7' * rotCBin6;
    rotCBin8' * rotCBin7;
    rotCBin9' * rotCBin8;
    rotCBin10' * rotCBin9;
    rotCBin1' * rotCBin10];


