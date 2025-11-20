<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Pine Lake Robotics Team Website Embed</title>
    <!-- Load Tailwind CSS -->
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        /* Custom styles to ensure the main content container fills the viewport height */
        .h-screen-minus-header {
            /* Calculate full viewport height minus the approximate header height (h-16) */
            height: calc(100vh - 4rem); 
        }
        /* Style for the iframe to ensure it is full size within its container */
        .full-iframe {
            width: 100%;
            height: 100%;
            border: none;
        }
    </style>
</head>
<body class="bg-gray-50 font-sans antialiased flex flex-col min-h-screen">
    <header class="bg-white shadow-md p-4 flex items-center justify-center h-16">
        <h1 class="text-xl sm:text-2xl font-extrabold text-indigo-700">
            Pine Lake Robotics Team Website
        </h1>
    </header>
    <!-- Main Content Area for the Iframe -->
    <main class="flex-grow p-4 md:p-6 lg:p-8 overflow-hidden">
        <div class="max-w-7xl mx-auto h-screen-minus-header bg-white rounded-xl shadow-2xl overflow-hidden border border-gray-200">
            <!-- The Iframe Element -->
            <iframe
                src="https://pinelakeroboticteam.lovable.app/"
                title="Embedded Pine Lake Robotics Team Website"
                class="full-iframe rounded-xl"
                loading="lazy"
                sandbox="allow-scripts allow-same-origin allow-popups allow-forms"
            >
                Your browser does not support iframes. Please visit the site directly: 
                <a href="https://pinelakeroboticteam.lovable.app/" target="_blank" rel="noopener noreferrer" class="text-indigo-500 hover:text-indigo-700">Pine Lake Robotics Team</a>
            </iframe>
        </div>
    </main>
    <!-- Footer for context -->
    <footer class="text-center p-2 text-sm text-gray-500 border-t">
        This content is displayed in an iframe from the source URL: https://pinelakeroboticteam.lovable.app/
    </footer>

</body>
</html>
