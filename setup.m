addpath('3rd-party', 'sample-maps', 'tests', 'tools')
mapMakerGUI;

% START Welcome Message
fontName = 'FixedWidth';
fontSize = 12;

msg_body = [ ...
    'Welcome to the MATLAB Simulator!' ...
    char(10) char(10) ...
    ];

msg_title = 'Welcome!';

msgbox(msg_body, msg_title);
% --- END Welcome Message