function toggleSignal(~, evnt)
if strcmp(evnt.Peer.Visible, 'on')
    evnt.Peer.Visible = 'off';
else
    evnt.Peer.Visible = 'on';
end
end