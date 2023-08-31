function phrase = affirmative()
    % List of light-hearted, fun, and/or funny affirmative phrases
    affirmativePhrases = ["On it!", "Moving now!", "Okay!", "Affirmative!", "Let's go!", "Roger that!", "You got it!", "Ready, set, go!", "Zipping through!", "Zooming away!", "Full speed ahead!", "No time to waste!", "Consider it done!", "Watch me go!", "Onwards and upwards!"];

    % Choose a random phrase from the list
    randomIndex = randi(length(affirmativePhrases));
    phrase = affirmativePhrases(randomIndex);
end
