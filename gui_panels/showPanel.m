function showPanel(panel)
    %SHOWPANEL Show all components in a panel
    if isfield(panel, 'components') && ~isempty(panel.components)
        validComponents = panel.components(isvalid(panel.components));
        if ~isempty(validComponents)
            set(validComponents, 'Visible', 'on');
        end
    end
end
