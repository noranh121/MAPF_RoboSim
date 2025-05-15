# tests/test_about_page.py
import pytest
from app import app
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import threading
import socket
from contextlib import closing


# Shared fixture for the Flask app
@pytest.fixture(scope="class")
def live_server():
    """Start Flask app once for all navigation tests"""
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(('', 0))
        s.listen(1)
        port = s.getsockname()[1]
    
    server_thread = threading.Thread(
        target=lambda: app.run(host='127.0.0.1', port=port, debug=False, use_reloader=False)
    )
    server_thread.daemon = True
    server_thread.start()
    time.sleep(2)
    
    yield f"http://127.0.0.1:{port}"


class TestAboutPageRoutes:
    """Test the about page route and basic functionality"""
    
    @pytest.fixture
    def client(self):
        """Create a test client for the Flask app"""
        app.config['TESTING'] = True
        with app.test_client() as client:
            yield client
    
    def test_about_route_loads(self, client):
        """Test that the about route '/about' loads successfully"""
        response = client.get('/about')
        assert response.status_code == 200
    
    def test_about_route_has_correct_title(self, client):
        """Test that the page has the correct title"""
        response = client.get('/about')
        assert b'<title>MAPF Simulator</title>' in response.data
    
    def test_about_page_has_main_content(self, client):
        """Test that main content exists"""
        response = client.get('/about')
        assert b'Multi-Agent-Path-Finding Algorithms' in response.data
        assert b'Multi-Agent Pathfinding (MAPF) problem' in response.data
        assert b'automated warehouses' in response.data
        assert b'autonomous vehicles' in response.data
    
    def test_about_page_has_navigation_header(self, client):
        """Test that the navigation header exists"""
        response = client.get('/about')
        assert b'RoboSim' in response.data
        assert b'nav' in response.data
    
    def test_about_page_has_buttons(self, client):
        """Test that both buttons exist"""
        response = client.get('/about')
        assert b'Read More' in response.data
        assert b'Benchmarks' in response.data


class TestAboutPageContent:
    """Test the content and structure of the about page"""
    
    @pytest.fixture(autouse=True)
    def setup_selenium(self, live_server):
        """Setup Selenium webdriver"""
        self.driver = webdriver.Chrome()
        self.driver.implicitly_wait(5)
        self.base_url = live_server
        
        yield
        self.driver.quit()
    
    def test_page_title_in_browser(self):
        """Test that the browser tab shows correct title"""
        self.driver.get(f"{self.base_url}/about")
        assert self.driver.title == "MAPF Simulator"
    
    def test_navigation_links_work(self):
        """Test all navigation menu links work from about page"""
        self.driver.get(f"{self.base_url}/about")
        
        nav_tests = [
            ("Main", "/"),
            ("Home", "/home"),
            ("Instructions", "/info")
        ]
        
        for link_text, expected_url_part in nav_tests:
            nav_link = WebDriverWait(self.driver, 10).until(
                EC.element_to_be_clickable((By.LINK_TEXT, link_text))
            )
            nav_link.click()
            
            time.sleep(1)
            
            assert expected_url_part in self.driver.current_url, f"Failed navigation to {link_text}"
            
            # Go back to about page for next test
            self.driver.get(f"{self.base_url}/about")
    
    def test_main_heading_exists(self):
        """Test that the main heading is present and correct"""
        self.driver.get(f"{self.base_url}/about")
        
        heading = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.TAG_NAME, "h1"))
        )
        assert "Multi-Agent-Path-Finding Algorithms" in heading.text
        assert heading.is_displayed()
    
    def test_gif_image_loads(self):
        """Test that the MAPF demo GIF loads correctly"""
        self.driver.get(f"{self.base_url}/about")
        
        gif_image = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "img[src*='mapf-demo.gif']"))
        )
        assert gif_image.is_displayed()
        assert "mapf-demo.gif" in gif_image.get_attribute("src")
        img_classes = gif_image.get_attribute("class")
        assert "img-fluid" in img_classes
        assert gif_image.get_attribute("loading") == "lazy"
        rendered_width = gif_image.get_attribute("width")
        rendered_height = gif_image.get_attribute("height")
        assert int(rendered_width) > 0
        assert int(rendered_height) > 0
    
    def test_content_description_exists(self):
        """Test that the description text is present"""
        self.driver.get(f"{self.base_url}/about")
        
        description = self.driver.find_element(By.CLASS_NAME, "lead")
        description_text = description.text
        
        assert "Multi-Agent Pathfinding (MAPF) problem" in description_text
        assert "automated warehouses" in description_text
        assert "autonomous vehicles" in description_text
        assert "planning paths for multiple agents" in description_text
    
    def test_read_more_button_functionality(self):
        """Test that the Read More button works"""
        self.driver.get(f"{self.base_url}/about")
        
        # Get initial window count
        initial_windows = len(self.driver.window_handles)
        
        # Find and click the Read More button
        read_more_button = WebDriverWait(self.driver, 10).until(
            EC.element_to_be_clickable((By.LINK_TEXT, "Read More"))
        )
        
        # Verify it has correct attributes
        assert read_more_button.get_attribute("href") == "https://mapf.info/"
        assert read_more_button.get_attribute("target") == "_blank"
        read_more_button.click()
        WebDriverWait(self.driver, 10).until(
            lambda driver: len(driver.window_handles) > initial_windows
        )
        
        # Verify new window opened
        assert len(self.driver.window_handles) > initial_windows
    
    def test_benchmarks_button_functionality(self):
        """Test that the Benchmarks button works"""
        self.driver.get(f"{self.base_url}/about")
        
        # Get initial window count
        initial_windows = len(self.driver.window_handles)
        
        # Find the Benchmarks button
        benchmarks_button = WebDriverWait(self.driver, 10).until(
            EC.element_to_be_clickable((By.LINK_TEXT, "Benchmarks"))
        )
        assert benchmarks_button.get_attribute("href") == "https://movingai.com/benchmarks/mapf/index.html"
        assert benchmarks_button.get_attribute("target") == "_blank"
        benchmarks_button.click()
        WebDriverWait(self.driver, 10).until(
            lambda driver: len(driver.window_handles) > initial_windows
        )
        
        # Verify new window opened
        assert len(self.driver.window_handles) > initial_windows
    
    def test_button_styling(self):
        """Test that buttons have correct CSS classes"""
        self.driver.get(f"{self.base_url}/about")
        
        # Check Read More button styling
        read_more_link = self.driver.find_element(By.LINK_TEXT, "Read More")
        read_more_button = read_more_link.find_element(By.TAG_NAME, "button")
        button_classes = read_more_button.get_attribute("class")
        
        assert "btn" in button_classes
        assert "btn-outline-light" in button_classes
        assert "btn-lg" in button_classes
        assert "button-color" in button_classes
        
        # Check Benchmarks button styling
        benchmarks_link = self.driver.find_element(By.LINK_TEXT, "Benchmarks")
        benchmarks_button = benchmarks_link.find_element(By.TAG_NAME, "button")
        button_classes = benchmarks_button.get_attribute("class")
        
        assert "btn" in button_classes
        assert "btn-outline-dark" in button_classes
        assert "btn-lg" in button_classes
    
    def test_logo_image_exists(self):
        """Test that the robot logo exists in the header"""
        self.driver.get(f"{self.base_url}/about")
        
        logo = WebDriverWait(self.driver, 10).until(
            EC.presence_of_element_located((By.CSS_SELECTOR, "header img"))
        )
        
        assert "icons8-robot-2-30.png" in logo.get_attribute("src")
    
    def test_bootstrap_grid_structure(self):
        """Test that Bootstrap grid structure is correctly applied"""
        self.driver.get(f"{self.base_url}/about")
        
        # Check for main container
        container = self.driver.find_element(By.CLASS_NAME, "container")
        assert container.is_displayed()
        
        # Check for row
        row = self.driver.find_element(By.CLASS_NAME, "row")
        assert row.is_displayed()
        
        # Check for grid columns
        columns = self.driver.find_elements(By.CSS_SELECTOR, "[class*='col-']")
        assert len(columns) >= 2  # Should have at least 2 columns