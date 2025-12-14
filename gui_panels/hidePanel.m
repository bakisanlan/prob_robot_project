function hidePanel(panel)
    %HIDEPANEL Hide all components in a panel
    if isfield(panel, 'components') && ~isempty(panel.components)
        validComponents = panel.components(isvalid(panel.components));
        if ~isempty(validComponents)
            set(validComponents, 'Visible', 'off');
        end
    end
end
