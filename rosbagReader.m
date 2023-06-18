%This function will read rosbag and all topics within it to store in single
%struct
function [topic_str] = rosbagReader(bagName)
bag = rosbag(bagName);
alltopics = bag.AvailableTopics;
topicNames = alltopics.Properties.RowNames;

for i = 1:length(topicNames)
    bagSelection = select(bag, 'Topic', topicNames{i});
    StructofMsg = readMessages(bagSelection,'DataFormat','struct'); 
    newchr = strrep(topicNames{i}, '/', '_');
    newchr = regexprep(newchr, '^_', '');
    topic_str.(newchr) = convert2struct(StructofMsg);
    
% for convenience, add one more field for the relative time in seconds
%mess1 = readMessages(bag,1);  % use the very first message as a reference



    if isfield(topic_str.(newchr), 'Header')

        zeroTimeSec = double(StructofMsg{1}.Header.Stamp.Sec);
        zeroTimeNsec = double(StructofMsg{1}.Header.Stamp.Nsec);
        % if the topic includes a Header, then use for the most accurate time
        topic_str.(newchr).RelTime = (double(topic_str.(newchr).Header.Stamp.Sec)-zeroTimeSec) + ...
                            (double(topic_str.(newchr).Header.Stamp.Nsec)-zeroTimeNsec)*1e-9;
    else 
        % We may lose a lot of accuracy in the time, so print a warning
        warning('No Header found for topic %s. RelTime may be inaccurate',topicNames{i});
        topic_str.(newchr).deltaSec = (bagSelection.MessageList.Time - zeroTimeSec);
        topic_str.(newchr).RelTime = topic_str.(newchr).deltaSec - zeroTimeNsec*1e-9;
    end


end




