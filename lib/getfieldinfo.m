function field = getfieldinfo()
% FIELDINFO

field.NUM_MARKERS = 6;

field.INNER_OFFSET_X = 32;
field.INNER_OFFSET_Y = 13;

field.INNER_SIZE_X = 420;
field.INNER_SIZE_Y = 270;

field.COMPLETE_SIZE_X = field.INNER_SIZE_X + 2 * field.INNER_OFFSET_X;
field.COMPLETE_SIZE_Y = field.INNER_SIZE_Y + 2 * field.INNER_OFFSET_Y;

field.MARKER_OFFSET_X = 21;
field.MARKER_OFFSET_Y = 0;

field.MARKER_DIST_X = 442;
field.MARKER_DIST_Y = 292;

field.MARKER_X_POS = [ ...
    field.MARKER_OFFSET_X ...
    field.MARKER_OFFSET_X + 0.5 * field.MARKER_DIST_X ...
    field.MARKER_OFFSET_X + field.MARKER_DIST_X ...
    field.MARKER_OFFSET_X + field.MARKER_DIST_X ...
    field.MARKER_OFFSET_X + 0.5 * field.MARKER_DIST_X ...
    field.MARKER_OFFSET_X ...
];

field.MARKER_Y_POS = [ ...
    field.MARKER_OFFSET_Y ...
    field.MARKER_OFFSET_Y ...
    field.MARKER_OFFSET_Y ...
    field.MARKER_OFFSET_Y + field.MARKER_DIST_Y ...
    field.MARKER_OFFSET_Y + field.MARKER_DIST_Y ...
    field.MARKER_OFFSET_Y + field.MARKER_DIST_Y ...
];
